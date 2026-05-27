#include "can_odom.h"

#include "fdcan.h"

#include "cmsis_os2.h"

#include <string.h>

/* ---- 수신 캐시 (인터럽트 컨텍스트에서 갱신) ---- */
static volatile float    g_cmd_linear_v  = 0.0f;
static volatile float    g_cmd_angular_w = 0.0f;
static volatile uint32_t g_cmd_last_rx_ms = 0U;

/* -------------------------------------------------- */

bool can_odom_init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    /* cmd_vel(0x200) ID 전용 수신 필터 → RxFifo0 */
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0U;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = CAN_ODOM_RX_ID;  /* 수신할 ID */
    filter.FilterID2    = 0x7FFU;          /* 마스크: 모든 비트 일치 */

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        return false;
    }

    /* 필터와 일치하지 않는 프레임은 거부 */
    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1,
            FDCAN_REJECT,
            FDCAN_REJECT,
            FDCAN_FILTER_REMOTE,
            FDCAN_FILTER_REMOTE) != HAL_OK) {
        return false;
    }

    /* RxFifo0에 새 메시지가 들어오면 IT0 라인으로 인터럽트 발생 */
    if (HAL_FDCAN_ActivateNotification(
            &hfdcan1,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
            0U) != HAL_OK) {
        return false;
    }

    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        return false;
    }

    return true;
}

bool can_odom_tx_odom(float linear_v, float angular_w)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8];

    tx_header.Identifier          = CAN_ODOM_TX_ID;
    tx_header.IdType              = FDCAN_STANDARD_ID;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_ON;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0U;

    memcpy(&tx_data[0], &linear_v,  sizeof(float));
    memcpy(&tx_data[4], &angular_w, sizeof(float));

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK;
}

void can_odom_get_cmd_vel(float *linear_v, float *angular_w, uint32_t now_ms)
{
    if (linear_v == NULL || angular_w == NULL) {
        return;
    }

    if ((now_ms - g_cmd_last_rx_ms) > CAN_ODOM_CMD_TIMEOUT_MS) {
        *linear_v  = 0.0f;
        *angular_w = 0.0f;
        return;
    }

    *linear_v  = g_cmd_linear_v;
    *angular_w = g_cmd_angular_w;
}

/* ---- HAL 콜백 (인터럽트 컨텍스트) ---- */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    /* ID 검증 및 DLC 검증 */
    if (rx_header.Identifier != CAN_ODOM_RX_ID) {
        return;
    }

    if (rx_header.DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }

    float lv, aw;
    memcpy(&lv, &rx_data[0], sizeof(float));
    memcpy(&aw, &rx_data[4], sizeof(float));

    g_cmd_linear_v   = lv;
    g_cmd_angular_w  = aw;
    /* osKernelGetTickCount는 ISR에서 호출 가능 */
    g_cmd_last_rx_ms = (uint32_t) osKernelGetTickCount();
}
