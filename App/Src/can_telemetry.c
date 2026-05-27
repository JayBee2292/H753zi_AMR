#include "can_telemetry.h"

#include "fdcan.h"

#include <limits.h>
#include <string.h>

enum {
    CAN_TELEMETRY_PERIOD_MS = 20U,
    CAN_TELEMETRY_TDC_OFFSET_TQ = 21U,
    CAN_TELEMETRY_TDC_FILTER_TQ = 0U,
    CAN_TELEMETRY_PAYLOAD_BYTES = 64U,
    CAN_TELEMETRY_CMD_ENABLE = 1U,
};

typedef struct {
    uint32_t sequence;
    uint32_t last_tx_ms;
    bool is_initialized;
    bool is_enabled;
} can_telemetry_state_t;

static can_telemetry_state_t g_can_telemetry = {
    .sequence = 0U,
    .last_tx_ms = 0U,
    .is_initialized = false,
    .is_enabled = true,
};

static int64_t can_telemetry_normalized_tick(const wheel_state_t *wheel)
{
    if (wheel == NULL) {
        return 0;
    }

    return wheel->is_inverted_direction ? -wheel->total_tick : wheel->total_tick;
}

static int16_t can_telemetry_i32_to_i16(int32_t value)
{
    if (value > (int32_t) INT16_MAX) {
        return INT16_MAX;
    }

    if (value < (int32_t) INT16_MIN) {
        return INT16_MIN;
    }

    return (int16_t) value;
}

static void can_telemetry_put_u32(uint8_t *buffer, uint32_t *offset, uint32_t value)
{
    memcpy(&buffer[*offset], &value, sizeof(value));
    *offset += (uint32_t) sizeof(value);
}

static void can_telemetry_put_u8(uint8_t *buffer, uint32_t *offset, uint8_t value)
{
    buffer[*offset] = value;
    *offset += (uint32_t) sizeof(value);
}

static void can_telemetry_put_i64(uint8_t *buffer, uint32_t *offset, int64_t value)
{
    memcpy(&buffer[*offset], &value, sizeof(value));
    *offset += (uint32_t) sizeof(value);
}

static void can_telemetry_put_i16(uint8_t *buffer, uint32_t *offset, int16_t value)
{
    memcpy(&buffer[*offset], &value, sizeof(value));
    *offset += (uint32_t) sizeof(value);
}

static void can_telemetry_put_f32(uint8_t *buffer, uint32_t *offset, float value)
{
    memcpy(&buffer[*offset], &value, sizeof(value));
    *offset += (uint32_t) sizeof(value);
}

static void can_telemetry_fill_payload(
    uint8_t *payload,
    const robot_status_t *robot,
    const can_telemetry_drive_state_t *drive,
    uint32_t now_ms)
{
    uint32_t offset = 0U;
    uint8_t flags = 0U;
    bool has_encoder_fault = false;

    memset(payload, 0, CAN_TELEMETRY_PAYLOAD_BYTES);

    if (robot->is_system_ready) {
        flags |= 0x01U;
    }

    if (drive->is_motion_active) {
        flags |= 0x02U;
    }

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        if (robot->wheels[i].is_fault) {
            has_encoder_fault = true;
            break;
        }
    }

    if (has_encoder_fault) {
        flags |= 0x04U;
    }

    can_telemetry_put_u32(payload, &offset, g_can_telemetry.sequence);
    can_telemetry_put_u32(payload, &offset, now_ms);
    can_telemetry_put_f32(payload, &offset, robot->robot_linear_v);
    can_telemetry_put_f32(payload, &offset, robot->robot_angular_w);

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        can_telemetry_put_f32(payload, &offset, robot->wheels[i].velocity_mps);
    }

    can_telemetry_put_i16(payload, &offset, can_telemetry_i32_to_i16(drive->left_duty_percent));
    can_telemetry_put_i16(payload, &offset, can_telemetry_i32_to_i16(drive->right_duty_percent));
    can_telemetry_put_i16(payload, &offset, can_telemetry_i32_to_i16(drive->duty_percent));
    can_telemetry_put_i16(payload, &offset, can_telemetry_i32_to_i16(drive->curve_ratio_percent));

    can_telemetry_put_u8(payload, &offset, drive->motion);
    can_telemetry_put_u8(payload, &offset, flags);
}

static void can_telemetry_fill_encoder_payload(
    uint8_t *payload,
    const robot_status_t *robot,
    uint32_t now_ms)
{
    uint32_t offset = 0U;
    uint8_t valid_mask = 0U;
    uint8_t fault_mask = 0U;

    memset(payload, 0, CAN_TELEMETRY_PAYLOAD_BYTES);

    can_telemetry_put_u32(payload, &offset, g_can_telemetry.sequence);
    can_telemetry_put_u32(payload, &offset, now_ms);

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        can_telemetry_put_i64(
            payload,
            &offset,
            can_telemetry_normalized_tick(&robot->wheels[i]));
    }

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        can_telemetry_put_u32(payload, &offset, robot->wheels[i].delta_tick_abs);
    }

    for (uint32_t i = 0U; i < WHEEL_COUNT; ++i) {
        uint8_t bit = (uint8_t) (1U << i);

        if (robot->wheels[i].is_fault) {
            fault_mask |= bit;
        } else {
            valid_mask |= bit;
        }
    }

    can_telemetry_put_u8(payload, &offset, valid_mask);
    can_telemetry_put_u8(payload, &offset, fault_mask);
}

static bool can_telemetry_tx_state(
    const robot_status_t *robot,
    const can_telemetry_drive_state_t *drive,
    uint32_t now_ms)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[CAN_TELEMETRY_PAYLOAD_BYTES];

    tx_header.Identifier = CAN_TELEMETRY_TX_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_64;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    can_telemetry_fill_payload(tx_data, robot, drive, now_ms);

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK;
}

static bool can_telemetry_tx_encoder(const robot_status_t *robot, uint32_t now_ms)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[CAN_TELEMETRY_PAYLOAD_BYTES];

    tx_header.Identifier = CAN_TELEMETRY_ENCODER_TX_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_64;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_ON;
    tx_header.FDFormat = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    can_telemetry_fill_encoder_payload(tx_data, robot, now_ms);

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) == HAL_OK;
}

bool can_telemetry_init(void)
{
    FDCAN_FilterTypeDef filter = {0};

    memset(&g_can_telemetry, 0, sizeof(g_can_telemetry));
    g_can_telemetry.is_enabled = true;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = CAN_TELEMETRY_RX_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        return false;
    }

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
            CAN_TELEMETRY_TDC_OFFSET_TQ,
            CAN_TELEMETRY_TDC_FILTER_TQ) != HAL_OK) {
        return false;
    }

    if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) {
        return false;
    }

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

    g_can_telemetry.last_tx_ms = 0U;
    g_can_telemetry.is_initialized = true;

    return true;
}

void can_telemetry_process(
    const robot_status_t *robot,
    const can_telemetry_drive_state_t *drive,
    uint32_t now_ms)
{
    if (!g_can_telemetry.is_initialized ||
        !g_can_telemetry.is_enabled ||
        robot == NULL ||
        drive == NULL) {
        return;
    }

    if ((now_ms - g_can_telemetry.last_tx_ms) < CAN_TELEMETRY_PERIOD_MS) {
        return;
    }

    if (can_telemetry_tx_state(robot, drive, now_ms) &&
        can_telemetry_tx_encoder(robot, now_ms)) {
        g_can_telemetry.last_tx_ms = now_ms;
        g_can_telemetry.sequence++;
    }
}

uint32_t can_telemetry_get_period_ms(void)
{
    return CAN_TELEMETRY_PERIOD_MS;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8] = {0};

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (rx_header.Identifier != CAN_TELEMETRY_RX_ID ||
        rx_header.DataLength != FDCAN_DLC_BYTES_8) {
        return;
    }

    if (rx_data[0] == CAN_TELEMETRY_CMD_ENABLE) {
        g_can_telemetry.is_enabled = rx_data[1] != 0U;
    }
}
