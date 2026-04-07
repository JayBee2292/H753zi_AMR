#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
    (void)transport;
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
    (void)transport;
    return true;
}

size_t cubemx_transport_write(
    struct uxrCustomTransport * transport,
    const uint8_t * buf,
    size_t len,
    uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;
    HAL_StatusTypeDef ret = HAL_UART_Transmit(uart, (uint8_t *) buf, len, 1000);

    if (ret != HAL_OK) {
        if (err != NULL) {
            *err = 1;
        }
        return 0;
    }

    return len;
}

size_t cubemx_transport_read(
    struct uxrCustomTransport * transport,
    uint8_t * buf,
    size_t len,
    int timeout,
    uint8_t * err)
{
    UART_HandleTypeDef * uart = (UART_HandleTypeDef *) transport->args;
    size_t received = 0;
    uint32_t start = HAL_GetTick();
    uint32_t budget_ms = (timeout < 0) ? 0U : (uint32_t) timeout;

    while (received < len) {
        uint32_t elapsed = HAL_GetTick() - start;
        if (received > 0 && elapsed >= budget_ms) {
            break;
        }

        uint32_t chunk_timeout = 1U;
        if (received == 0 && budget_ms > 0U) {
            chunk_timeout = budget_ms;
        }

        HAL_StatusTypeDef ret = HAL_UART_Receive(uart, &buf[received], 1, chunk_timeout);
        if (ret == HAL_OK) {
            received++;
            continue;
        }

        if (ret != HAL_TIMEOUT) {
            if (err != NULL) {
                *err = 1;
            }
            break;
        }

        if ((HAL_GetTick() - start) >= budget_ms) {
            break;
        }
    }

    return received;
}

#endif
