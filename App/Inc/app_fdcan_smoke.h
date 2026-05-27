#ifndef APP_FDCAN_SMOKE_H
#define APP_FDCAN_SMOKE_H

#include <stdbool.h>
#include <stdint.h>

#include "usart.h"

/* CAN FD test frame:
 * - Standard ID: 0x321
 * - DLC: 8
 * - Byte[0..3]: little-endian uint32 counter
 * - Byte[4..7]: reserved (0)
 * Counter sequence: 1, 2, 3, ... UINT32_MAX, 1, 2, ... */
#define APP_FDCAN_SMOKE_TX_ID 0x321U

bool app_fdcan_smoke_init(UART_HandleTypeDef *debug_uart);
void app_fdcan_smoke_process(uint32_t now_ms);
uint32_t app_fdcan_smoke_get_period_ms(void);

#endif /* APP_FDCAN_SMOKE_H */
