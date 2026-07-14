#include "can_telemetry.h"

#include "fdcan.h"

#include <limits.h>
#include <string.h>

enum {
    CAN_TELEMETRY_PERIOD_MS = 40U,
    CAN_TELEMETRY_TDC_OFFSET_TQ = 21U,
    CAN_TELEMETRY_TDC_FILTER_TQ = 0U,
    CAN_TELEMETRY_PAYLOAD_BYTES = 64U,
    CAN_TELEMETRY_CMD_ENABLE = 1U,
    CAN_TELEMETRY_STATE_SLOT = 4U,
    CAN_TELEMETRY_SLOT_COUNT = 5U,
};

typedef struct {
    uint32_t sequence;
    uint32_t last_tx_ms;
    volatile uint32_t bus_off_count;
    volatile uint32_t error_event_count;
    volatile uint32_t last_error_status_its;
    volatile uint8_t last_error_code;
    volatile uint8_t data_last_error_code;
    volatile uint8_t tx_error_count;
    volatile uint8_t rx_error_count;
    volatile uint8_t error_logging_count;
    volatile uint8_t error_passive;
    volatile uint8_t error_warning;
    volatile uint8_t protocol_exception;
    volatile uint8_t tdc_value;
    uint8_t schedule_slot;
    bool is_initialized;
    volatile bool is_enabled;
} can_telemetry_state_t;

static can_telemetry_state_t g_can_telemetry = {
    .sequence = 0U,
    .last_tx_ms = 0U,
    .bus_off_count = 0U,
    .schedule_slot = 0U,
    .is_initialized = false,
    .is_enabled = false,
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
    can_telemetry_put_u32(payload, &offset, g_can_telemetry.bus_off_count);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.last_error_code);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.data_last_error_code);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.tx_error_count);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.rx_error_count);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.error_logging_count);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.error_passive);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.error_warning);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.protocol_exception);
    can_telemetry_put_u8(payload, &offset, g_can_telemetry.tdc_value);
    can_telemetry_put_u32(payload, &offset, g_can_telemetry.error_event_count);
    can_telemetry_put_u32(payload, &offset, g_can_telemetry.last_error_status_its);
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
    g_can_telemetry.is_enabled = false;

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
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_ERROR_WARNING |
            FDCAN_IT_ERROR_PASSIVE |
            FDCAN_IT_BUS_OFF,
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

    bool sent = false;

    if (g_can_telemetry.schedule_slot == CAN_TELEMETRY_STATE_SLOT) {
        sent = can_telemetry_tx_state(robot, drive, now_ms);
    } else {
        sent = can_telemetry_tx_encoder(robot, now_ms);
    }

    if (sent) {
        g_can_telemetry.schedule_slot =
            (uint8_t) ((g_can_telemetry.schedule_slot + 1U) % CAN_TELEMETRY_SLOT_COUNT);
        g_can_telemetry.last_tx_ms = now_ms;
        g_can_telemetry.sequence++;
    }
}

uint32_t can_telemetry_get_period_ms(void)
{
    return CAN_TELEMETRY_PERIOD_MS;
}

void can_telemetry_get_diagnostics(can_telemetry_diagnostics_t *diagnostics)
{
    if (diagnostics == NULL) {
        return;
    }

    diagnostics->bus_off_count = g_can_telemetry.bus_off_count;
    diagnostics->error_event_count = g_can_telemetry.error_event_count;
    diagnostics->last_error_status_its = g_can_telemetry.last_error_status_its;
    diagnostics->last_error_code = g_can_telemetry.last_error_code;
    diagnostics->data_last_error_code = g_can_telemetry.data_last_error_code;
    diagnostics->tx_error_count = g_can_telemetry.tx_error_count;
    diagnostics->rx_error_count = g_can_telemetry.rx_error_count;
    diagnostics->error_logging_count = g_can_telemetry.error_logging_count;
    diagnostics->error_passive = g_can_telemetry.error_passive;
    diagnostics->error_warning = g_can_telemetry.error_warning;
    diagnostics->protocol_exception = g_can_telemetry.protocol_exception;
    diagnostics->tdc_value = g_can_telemetry.tdc_value;
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    FDCAN_ProtocolStatusTypeDef protocol_status = {0};
    FDCAN_ErrorCountersTypeDef error_counters = {0};

    if (hfdcan->Instance != FDCAN1) {
        return;
    }

    (void) HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
    (void) HAL_FDCAN_GetErrorCounters(hfdcan, &error_counters);

    if (protocol_status.Warning != 0U || protocol_status.ErrorPassive != 0U) {
        g_can_telemetry.last_error_code = (uint8_t) protocol_status.LastErrorCode;
        g_can_telemetry.data_last_error_code = (uint8_t) protocol_status.DataLastErrorCode;
        g_can_telemetry.tx_error_count = (uint8_t) error_counters.TxErrorCnt;
        g_can_telemetry.rx_error_count = (uint8_t) error_counters.RxErrorCnt;
        g_can_telemetry.error_logging_count = (uint8_t) error_counters.ErrorLogging;
        g_can_telemetry.error_passive = (uint8_t) protocol_status.ErrorPassive;
        g_can_telemetry.error_warning = (uint8_t) protocol_status.Warning;
        g_can_telemetry.protocol_exception = (uint8_t) protocol_status.ProtocolException;
        g_can_telemetry.tdc_value = (uint8_t) protocol_status.TDCvalue;
        g_can_telemetry.last_error_status_its = ErrorStatusITs;
        g_can_telemetry.error_event_count++;
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U) {
        g_can_telemetry.bus_off_count++;
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[CAN_TELEMETRY_PAYLOAD_BYTES] = {0};

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
