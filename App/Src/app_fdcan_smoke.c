#include "app_fdcan_smoke.h"

#include "fdcan.h"

#include <stdio.h>
#include <string.h>

enum {
    APP_FDCAN_SMOKE_TX_PERIOD_MS = 10U,
    APP_FDCAN_SMOKE_PRINT_PERIOD_MS = 200U,
    APP_FDCAN_SMOKE_TDC_OFFSET_TQ = 21U,
    APP_FDCAN_SMOKE_TDC_FILTER_TQ = 0U,
};

typedef struct {
    UART_HandleTypeDef *debug_uart;
    uint32_t counter;
    uint32_t last_tx_ms;
    uint32_t last_print_ms;
    bool is_initialized;
} app_fdcan_smoke_state_t;

static app_fdcan_smoke_state_t g_app_fdcan_smoke = {0};

static uint32_t app_fdcan_smoke_next_counter(uint32_t current)
{
    if (current == 0U || current == UINT32_MAX) {
        return 1U;
    }

    return current + 1U;
}

static bool app_fdcan_smoke_tx_counter(uint32_t counter)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};

    tx_header.Identifier = APP_FDCAN_SMOKE_TX_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    memcpy(&tx_data[0], &counter, sizeof(counter));

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK;
}

static void app_fdcan_smoke_print_status(uint32_t counter)
{
    char line[96];
    int length;

    if (g_app_fdcan_smoke.debug_uart == NULL) {
        return;
    }

    length = snprintf(
        line,
        sizeof(line),
        "FDCAN FD+BRS tx id=0x%03lX counter=%lu\r\n",
        (unsigned long) APP_FDCAN_SMOKE_TX_ID,
        (unsigned long) counter);

    if (length <= 0) {
        return;
    }

    HAL_UART_Transmit(
        g_app_fdcan_smoke.debug_uart,
        (uint8_t *) line,
        (uint16_t) length,
        HAL_MAX_DELAY);
}

bool app_fdcan_smoke_init(UART_HandleTypeDef *debug_uart)
{
    memset(&g_app_fdcan_smoke, 0, sizeof(g_app_fdcan_smoke));
    g_app_fdcan_smoke.debug_uart = debug_uart;

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_ConfigTxDelayCompensation(
            &hfdcan1,
            APP_FDCAN_SMOKE_TDC_OFFSET_TQ,
            APP_FDCAN_SMOKE_TDC_FILTER_TQ) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }

    g_app_fdcan_smoke.last_tx_ms = HAL_GetTick();
    g_app_fdcan_smoke.last_print_ms = g_app_fdcan_smoke.last_tx_ms;
    g_app_fdcan_smoke.is_initialized = true;

    return true;
}

void app_fdcan_smoke_process(uint32_t now_ms)
{
    uint32_t counter;

    if (!g_app_fdcan_smoke.is_initialized) {
        return;
    }

    if ((now_ms - g_app_fdcan_smoke.last_tx_ms) < APP_FDCAN_SMOKE_TX_PERIOD_MS) {
        return;
    }

    counter = app_fdcan_smoke_next_counter(g_app_fdcan_smoke.counter);

    if (!app_fdcan_smoke_tx_counter(counter)) {
        return;
    }

    g_app_fdcan_smoke.counter = counter;
    g_app_fdcan_smoke.last_tx_ms = now_ms;

    if ((now_ms - g_app_fdcan_smoke.last_print_ms) >= APP_FDCAN_SMOKE_PRINT_PERIOD_MS) {
        app_fdcan_smoke_print_status(counter);
        g_app_fdcan_smoke.last_print_ms = now_ms;
    }
}

uint32_t app_fdcan_smoke_get_period_ms(void)
{
    return APP_FDCAN_SMOKE_TX_PERIOD_MS;
}
