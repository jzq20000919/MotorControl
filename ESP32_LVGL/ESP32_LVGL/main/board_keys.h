#ifndef BOARD_KEYS_H
#define BOARD_KEYS_H

#include <stdint.h>

#define BOARD_KEY_K0  (1U << 0)
#define BOARD_KEY_K1  (1U << 1)
#define BOARD_KEY_K2  (1U << 2)

/** @brief 配置三个按键对应的 GPIO 与 I/O 扩展器引脚。 */
void board_keys_init(void);
/** @brief 读取按键即时状态，返回 BOARD_KEY_Kx 标志位的按位或。 */
uint8_t board_keys_read(void);

#endif /* BOARD_KEYS_H */
