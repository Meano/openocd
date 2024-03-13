#ifndef OPENOCD_FLASH_NOR_UFL_H
#define OPENOCD_FLASH_NOR_UFL_H

#define UFL_MAX_CHUNK_SIZE 65536

#define UFL_WAIT_TARGET_TIMEOUT 1000

#define UFL_PROGRAM         0x474F5250L
#define UFL_ERASE           0x00415245L
#define UFL_ERASE_ALL       0x4C415245L
#define UFL_INFO            0x4F464E49L

#endif /* OPENOCD_FLASH_NOR_UFL_H */
