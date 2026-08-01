#ifndef BOARD_KEYS_H
#define BOARD_KEYS_H

#include <stdint.h>

#define BOARD_KEY_K0  (1U << 0)
#define BOARD_KEY_K1  (1U << 1)
#define BOARD_KEY_K2  (1U << 2)

void board_keys_init(void);
uint8_t board_keys_read(void);

#endif /* BOARD_KEYS_H */
