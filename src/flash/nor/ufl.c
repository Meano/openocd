/***************************************************************************
 *   Copyright (C) 2021 by Meano                                           *
 *   meanocat@gmail.com                                                    *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "ufl.h"
#include <target/image.h>
#include <helper/time_support.h>

struct ufl {
	struct target *target;
	struct image image;
	uint32_t magic;
	uint32_t timeout;
	uint32_t command_addr;
	uint32_t command_size;
	uint32_t* command;
	uint32_t data_addr;
	uint32_t data_size;
	uint8_t* data;
	bool loaded;
};

// flash bank <flash#> ufl <bank address> <bank size> 0 0 <target#> <loader.elf> [Timeout:ms] [Magic:u32]
FLASH_BANK_COMMAND_HANDLER(ufl_flash_bank_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct ufl* ufl = bank->driver_priv;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ufl = calloc(sizeof(struct ufl), 1);

	struct image *image = &ufl->image;

	int ret = image_open(image, CMD_ARGV[6], "elf");
	if (ret != ERROR_OK)
		goto Failed;

	ufl->timeout = 30000; // defalut timeout: 30000ms(30s)
	if (CMD_ARGC > 7)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], ufl->timeout);

	ufl->magic = 0x4D4C463D;
	if (CMD_ARGC > 8)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], ufl->magic);

	image->base_address = image->sections[0].base_address;

	uint32_t offset = 0;
	uint32_t magic = 0;
	for (uint32_t i = 0; i < image->num_sections; i++) {
		struct imagesection* section = &image->sections[i];
		if (offset <= 0x100 && (offset + section->size) > 0x110) {
			uint32_t loaderInfo[5] = { 0 };
			size_t loaderSize = 0;
			image_read_section(image, i, 0x100 - offset, 5 * 4, (uint8_t *)loaderInfo, &loaderSize);
			if (loaderSize == 20 && loaderInfo[0] == ufl->magic) {
				magic = loaderInfo[0];
				ufl->command_addr = loaderInfo[1];
				ufl->command_size = loaderInfo[2];
				ufl->data_addr = loaderInfo[3]; 
				ufl->data_size = loaderInfo[4];
				break;
			}
			else {
				goto Failed;
			}
		}
		offset += section->size;
	}

	if (magic != ufl->magic) {
		goto Failed;
	}

	ufl->loaded = false;
	ufl->command = calloc(4, ufl->command_size / 4);
	ufl->data = calloc(4, ufl->data_size / 4);

	if (ufl->command == NULL || ufl->data == NULL) {
		LOG_ERROR("ufl: Command or Data alloc failed!");
		goto Failed;
	}

	bank->driver_priv = ufl;
	ufl->target = target;
	LOG_DEBUG("ufl: The file is ufl Loader!");
	return ERROR_OK;

Failed:
	image_close(image);
	free(ufl->command);
	free(ufl->data);
	free(ufl);
	bank->driver_priv = NULL;
	LOG_ERROR("ufl: The file is not ufl loder!");
	return ERROR_FAIL;
}

void ufl_free(struct flash_bank *bank)
{
	struct ufl* ufl = bank->driver_priv;

	if (ufl) {
		image_close(&ufl->image);
		free(ufl->command);
		free(ufl->data);
		free(ufl);
	}

	bank->driver_priv = NULL;
}


static int wait_target(struct target *target, bool isHalt, const char *action, uint32_t *address) {
	const char *targetAction = isHalt ? "halt" : "resume";
	enum target_state state = isHalt ? TARGET_HALTED : TARGET_RUNNING;

	int ret = ERROR_OK;
	if (target->state == state)
		return ret;

	ret = isHalt ? target_halt(target) : target_resume(target, address ? 0 : 1, address ? *address : 0, 0, 0);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to %s when %s.", targetAction, action);
		return ret;
	}

	ret = target_wait_state(target, state, UFL_WAIT_TARGET_TIMEOUT);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to wait %s when %s.", targetAction, action);
		return ret;
	}

	return ret;
}

static int wait_target_halt(struct target *target, const char *action) {
	return wait_target(target, true, action, NULL);
}

static int wait_target_resume(struct target *target, const char *action) {
	return wait_target(target, false, action, NULL);
}

static int wait_target_start(struct target *target, const char *action, uint32_t address) {
	return wait_target(target, false, action, &address);
}

static int ufl_wait_ready(struct ufl *ufl) {
	struct target *target = ufl->target;

	int ret = ERROR_FAIL;

	uint32_t command_offset = ufl->command_addr;
	uint32_t command_status = 0;

	int64_t start_ms = timeval_ms();

	while (1) {
		ret = wait_target_halt(target, "wait ready");
		if (ret != ERROR_OK) {
			goto Failed;
		}

		ret = target_read_u32(target, command_offset, &command_status);
		if (ret != ERROR_OK) {
			LOG_ERROR("ufl: Read status failed %08X", command_offset);
			goto Failed;
		}

		if (command_status & 0x40000000) {
			LOG_DEBUG("ufl: Status ready!");
			break;
		}

		ret = wait_target_resume(target, "wait ready");
		if (ret != ERROR_OK) {
			goto Failed;
		}

		alive_sleep(100);

		uint32_t elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > ufl->timeout) {
			LOG_DEBUG("ufl: Wait ready timeout %d ms (limit: %d ms)!", elapsed_ms, ufl->timeout);
			ret = ERROR_TIMEOUT;
			goto Failed;
		}
	}

	ret = target_read_memory(target, command_offset, 4, ufl->command_size / 4, (uint8_t *)ufl->command);
	if (ret != ERROR_OK) {
		LOG_DEBUG("ufl: Read params failed!");
		goto Failed;
	}

	LOG_DEBUG("ufl: Params %08X %08X %08X %08X", ufl->command[0], ufl->command[1], ufl->command[2], ufl->command[3]);

	ret = target_write_u32(target, command_offset, 0x00000000);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to clear wait.");
		goto Failed;
	}

	LOG_DEBUG("ufl: Command result %08X", command_status);

	ret = (int)command_status & 0xFFFF;

Failed:
	if (wait_target_halt(target, "end wait ready")) {
		return ERROR_FAIL;
	}

	return ret;
}

static int ufl_exec_command(struct ufl *ufl, uint32_t *command, uint32_t count, uint8_t *data, size_t size)
{
	struct target *target = ufl->target;

	int ret = ERROR_OK;

	ret = wait_target_halt(target, "exec command");
	if (ret != ERROR_OK) {
		return ret;
	}

	uint32_t data_offset = ufl->data_addr;

	if (data && size) {
		ret = target_write_memory(target, data_offset, 4, (size + 3) / 4, data);
		if (ret != ERROR_OK) {
			LOG_ERROR("ufl: Failed to load data!");
			return ret;
		}
	}

	uint32_t command_offset = ufl->command_addr;

	ret = target_write_memory(target, command_offset + 4, 4, count + 1, (uint8_t *)command);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to write command %08X!", *command);
		return ret;
	}

	ret = target_write_u32(target, command_offset, 0x80000000);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to write status ready %08X!", *command);
		return ret;
	}

	ret = ufl_wait_ready(ufl);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to wait command done!");
		return ret;
	}

	return ret;
}

static int ufl_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct ufl *ufl = bank->driver_priv;

	int ret = ERROR_OK;

	LOG_INFO("ufl: Starting erasing sector:%d -> sector:%d ...", first, last);
	uint32_t command[3];
	command[0] = UFL_ERASE;
	command[1] = first;
	command[2] = last + 1 - first;
	ret = ufl_exec_command(ufl, command, 2, NULL, 0);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to get flash info!");
		return ret;
	}

	return ret;
}

static int ufl_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct ufl *ufl = bank->driver_priv;

	int ret = ERROR_OK;

	uint32_t maxSize = ufl->data_size & 0xFFFFFFFC;

	while (count) {
		uint32_t writeSize = maxSize < count ? maxSize : count;

		memcpy(ufl->data, buffer, writeSize);

		LOG_INFO("ufl: Starting write offset %d, size %d ...", offset, writeSize);
		uint32_t command[3];
		command[0] = UFL_PROGRAM;
		command[1] = offset;
		command[2] = writeSize;
		ret = ufl_exec_command(ufl, command, 2, ufl->data, writeSize);
		if (ret != ERROR_OK) {
			LOG_ERROR("ufl: Failed to write flash!");
			return ret;
		}

		buffer += writeSize;
		offset += writeSize;
		count -= writeSize;
	}

	LOG_INFO("ufl: Write flash offset %d, size %d done!", offset, count);

	return ret;
}

static int ufl_probe(struct flash_bank *bank)
{
	struct ufl *ufl = bank->driver_priv;
	struct target *target = ufl->target;
	struct image *image = &ufl->image;

	int ret = ERROR_OK;

	ret = wait_target_halt(target, "start probe");
	if (ret != ERROR_OK) {
		return ret;
	}

	if (ufl->loaded) {
		uint32_t magic = 0;
		ret = target_read_memory(target, image->base_address + 0x100, 4, 1, (uint8_t *)&magic);
		if (ret != ERROR_OK) {
			LOG_ERROR("ufl: Read magic failed.");
			return ERROR_FAIL;
		}
		if (magic != ufl->magic) {
			ufl->loaded = false;
		}
		else {
			LOG_INFO("ufl: Flash loader already loaded!");
		}
	}

	uint8_t chunk[UFL_MAX_CHUNK_SIZE];

	if (!ufl->loaded) {
		LOG_INFO("ufl: Loading flash loader...");
		for (uint32_t i = 0; i < image->num_sections; i++) {
			struct imagesection* section = &image->sections[i];

			size_t readSize = 0;
			uint32_t readOffset = 0;
			do {
				LOG_DEBUG("ufl: Section size: %08X, address: %llX.", section->size, section->base_address);
				uint32_t size = section->size - readOffset;
				size = size > UFL_MAX_CHUNK_SIZE ? UFL_MAX_CHUNK_SIZE : size;

				ret = image_read_section(image, i, readOffset, size, chunk, &readSize);
				if ((ret != ERROR_OK) || (readSize != size)) {
					return ERROR_FAIL;
				}

				ret = target_write_memory(target, section->base_address + readOffset, 4, (size + 3) / 4, chunk);
				if (ret != ERROR_OK) {
					return ERROR_FAIL;
				}

				readOffset += readSize;
			}
			while(section->size > readOffset);
		}
	}

	ret = target_write_u32(target, ufl->command_addr, 0x00000000);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to clear start ready.");
		return ret;
	}

	LOG_INFO("ufl: Loaded flash loader! Starting...");
	ret = wait_target_start(target, "start loader", image->start_address);
	if (ret != ERROR_OK) {
		return ret;
	}

	ret = ufl_wait_ready(ufl);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to wait loader inited!");
		return ret;
	}

	if (ufl->loaded) {
		LOG_INFO("ufl: Flash probe again done!");
		return ERROR_OK;
	}

	uint32_t command[1];
	command[0] = UFL_INFO;
	ret = ufl_exec_command(ufl, command, 0, NULL, 0);
	if (ret != ERROR_OK) {
		LOG_ERROR("ufl: Failed to get flash info!");
		return ret;
	}
	LOG_INFO("ufl: flash info %08X %08X %08X!", ufl->command[2], ufl->command[3], ufl->command[4]);

	bank->size = ufl->command[2];
	uint32_t sectorSize = ufl->command[3];

	if ((bank->size == 0) || (bank->size % sectorSize)) {
		LOG_ERROR("Bank invalid size: %d, sector size: %d.", bank->size, sectorSize);
		return ERROR_FLASH_BANK_INVALID;
	}

	bank->num_sectors = bank->size / sectorSize;

	bank->sectors = realloc(bank->sectors, sizeof(struct flash_sector)*bank->num_sectors);
	for (uint32_t i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * sectorSize;
		bank->sectors[i].size = sectorSize;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	ufl->loaded = true;
	LOG_INFO("ufl: Flash probe done!");

	return ERROR_OK;
}

static int ufl_auto_probe(struct flash_bank *bank) {
	return ufl_probe(bank);
}

const struct flash_driver ufl_flash = {
	.name = "ufl",
	.flash_bank_command = ufl_flash_bank_command,
	.usage = "flash bank <flash#> ufl <bank address> <bank size> 0 0 <target#> <loader.elf> [Timeout:ms] [Magic:u32]",
	.erase = ufl_erase,
	.write = ufl_write,
	.read = default_flash_read,
	.probe = ufl_probe,
	.erase_check = default_flash_blank_check,
	.auto_probe = ufl_auto_probe,
	.free_driver_priv = ufl_free,
};
