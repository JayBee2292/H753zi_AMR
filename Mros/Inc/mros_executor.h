#ifndef MROS_EXECUTOR_H
#define MROS_EXECUTOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "usart.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define MROS_EXECUTOR_DEFAULT_NODE_NAME "cubemx_node" /* node_name 인자가 NULL일 때 사용할 기본 ROS 2 노드 이름 */

/**
 * @brief micro-ROS executor 초기화 컨텍스트
 * @details
 * - allocator/support/node/executor 수명주기를 한 구조체로 묶어 초기화 순서를 단순화합니다.
 * - `initialized`는 support/node 준비 여부, `executor_initialized`는 handle 기반 executor 준비 여부를 뜻합니다.
 */
typedef struct {
    rcl_allocator_t allocator;      /* micro-ROS에서 사용할 사용자 지정 allocator */
    rclc_support_t support;         /* rcl 초기화와 context를 보관하는 support 객체 */
    rcl_node_t node;                /* publish/subscribe의 부모가 되는 ROS 2 node */
    rclc_executor_t executor;       /* subscription callback dispatch용 executor */
    bool initialized;               /* support와 node 초기화 완료 여부 */
    bool executor_initialized;      /* executor handle pool 초기화 완료 여부 */
} mros_executor_context_t;

/**
 * @brief UART 기반 custom transport와 ROS 2 node를 초기화합니다.
 * @param context 초기화할 executor 컨텍스트
 * @param uart XRCE-DDS 전송에 사용할 UART
 * @param node_name 사용할 ROS 2 node 이름. NULL이면 기본값 사용
 * @retval 성공 시 true
 */
bool mros_executor_init(
    mros_executor_context_t *context,
    UART_HandleTypeDef *uart,
    const char *node_name);

/**
 * @brief executor를 특정 handle 수로 초기화합니다.
 * @details
 * - subscription을 executor에 attach하기 전에 필요한 최대 handle 수를 한 번 예약합니다.
 * - 이미 초기화된 경우에는 현재 capacity가 충분한지만 검사합니다.
 * @param context 준비할 executor 컨텍스트
 * @param handle_count 필요한 최대 handle 수
 * @retval 준비 성공 시 true
 */
bool mros_executor_prepare(
    mros_executor_context_t *context,
    size_t handle_count);

/**
 * @brief executor를 짧게 spin 하여 수신 callback을 처리합니다.
 * @param context spin 대상 executor 컨텍스트
 * @param timeout_ns spin 대기 시간(ns)
 * @retval 호출 성공 시 true
 */
bool mros_executor_spin_some(
    mros_executor_context_t *context,
    uint64_t timeout_ns);

#endif
