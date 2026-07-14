#ifndef CAN_TELEMETRY_H
#define CAN_TELEMETRY_H

#include "app_robot.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * CAN FD telemetry protocol, FDCAN1 FD+BRS (1 Mbps nominal / 5 Mbps data):
 *
 *   0x100  STM32 -> host, 64 bytes
 *          u32 seq
 *          u32 timestamp_ms
 *          f32 robot_linear_v_mps
 *          f32 robot_angular_w_radps
 *          f32 wheel_velocity_mps[4]  // FL, FR, RL, RR
 *          i16 left_duty_percent
 *          i16 right_duty_percent
 *          i16 duty_percent
 *          i16 curve_ratio_percent
 *          u8  motion
 *          u8  flags                  // bit0: robot ready, bit1: motion active, bit2: encoder fault
 *          u32 bus_off_count
 *          u8  last_error_code         // latest warning/passive PSR.LEC snapshot
 *          u8  data_last_error_code    // latest warning/passive PSR.DLEC snapshot
 *          u8  tx_error_count          // latest warning/passive ECR.TEC snapshot
 *          u8  rx_error_count          // latest warning/passive ECR.REC snapshot
 *          u8  error_logging_count     // latest warning/passive ECR.CEL snapshot
 *          u8  error_passive           // latest warning/passive PSR.EP snapshot
 *          u8  error_warning           // latest warning/passive PSR.EW snapshot
 *          u8  protocol_exception      // latest warning/passive PSR.PXE snapshot
 *          u8  tdc_value               // latest warning/passive PSR.TDCV snapshot
 *          u32 error_event_count        // warning/passive diagnostic snapshots
 *          u32 last_error_status_its    // FDCAN error-status interrupt bits
 *          u8  reserved[1]
 *
 *   0x101  STM32 -> host, 64 bytes
 *          u32 seq
 *          u32 timestamp_ms
 *          i64 encoder_tick[4]        // FL, FR, RL, RR, forward-normalized
 *          u32 encoder_delta_abs[4]   // abs(current_tick - previous_tick)
 *          u8  encoder_valid_mask     // bit0..3: FL, FR, RL, RR valid
 *          u8  encoder_fault_mask     // bit0..3: FL, FR, RL, RR rejected last sample
 *          u8  reserved[6]
 *
 *   0x1FF  host -> STM32, 8 bytes
 *          byte0 command
 *          byte1 value
 *          command 1: telemetry enable, value 0=disable, nonzero=enable
 */

#define CAN_TELEMETRY_TX_ID 0x100U
#define CAN_TELEMETRY_ENCODER_TX_ID 0x101U
#define CAN_TELEMETRY_RX_ID 0x1FFU

typedef struct {
    int32_t duty_percent;
    int32_t left_duty_percent;
    int32_t right_duty_percent;
    int32_t curve_ratio_percent;
    uint8_t motion;
    bool is_motion_active;
} can_telemetry_drive_state_t;

typedef struct {
    uint32_t bus_off_count;
    uint32_t error_event_count;
    uint32_t last_error_status_its;
    uint8_t last_error_code;
    uint8_t data_last_error_code;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
    uint8_t error_logging_count;
    uint8_t error_passive;
    uint8_t error_warning;
    uint8_t protocol_exception;
    uint8_t tdc_value;
} can_telemetry_diagnostics_t;

bool can_telemetry_init(void);
void can_telemetry_process(
    const robot_status_t *robot,
    const can_telemetry_drive_state_t *drive,
    uint32_t now_ms);
uint32_t can_telemetry_get_period_ms(void);
void can_telemetry_get_diagnostics(can_telemetry_diagnostics_t *diagnostics);

#endif /* CAN_TELEMETRY_H */
