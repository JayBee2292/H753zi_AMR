#include "mros_executor.h"

#include "cmsis_os2.h"

#include <string.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

/* Custom transport 구현체는 microros transport 레이어에서 제공합니다. */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(
    struct uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *err);
size_t cubemx_transport_read(
    struct uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *err);

void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);

/**
 * @brief micro-ROS 기본 allocator를 프로젝트 전용 allocator로 교체합니다.
 * @param allocator 설정 결과를 반환받을 allocator 포인터
 * @retval 설정 성공 시 true
 */
static bool mros_executor_configure_allocator(rcl_allocator_t *allocator)
{
    rcl_allocator_t freertos_allocator = rcutils_get_zero_initialized_allocator();

    freertos_allocator.allocate = microros_allocate;
    freertos_allocator.deallocate = microros_deallocate;
    freertos_allocator.reallocate = microros_reallocate;
    freertos_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freertos_allocator)) {
        return false;
    }

    *allocator = rcl_get_default_allocator();
    return true;
}

/**
 * @brief NULL 이름 인자를 기본 문자열로 치환합니다.
 * @param value 사용자 입력 문자열
 * @param fallback 기본 문자열
 * @retval 사용할 최종 문자열 포인터
 */
static const char *mros_executor_resolve_name(const char *value, const char *fallback)
{
    return (value != NULL) ? value : fallback;
}

/**
 * @brief UART custom transport와 ROS 2 node를 초기화합니다.
 * @details
 * - 전달된 UART는 raw XRCE-DDS transport로 전용 사용됩니다.
 * - agent 연결이 늦게 올라오는 상황을 허용하기 위해 support 초기화는 성공할 때까지 재시도합니다.
 * @param context 초기화할 executor 컨텍스트
 * @param uart XRCE-DDS transport로 사용할 UART
 * @param node_name 생성할 node 이름. NULL이면 기본 이름 사용
 * @retval 성공 시 true
 */
bool mros_executor_init(
    mros_executor_context_t *context,
    UART_HandleTypeDef *uart,
    const char *node_name)
{
    rcl_ret_t rc = RCL_RET_OK;

    if (context == NULL || uart == NULL) {
        return false;
    }

    memset(context, 0, sizeof(*context));
    context->node = rcl_get_zero_initialized_node();
    context->executor = rclc_executor_get_zero_initialized_executor();

    /* The selected UART becomes a raw XRCE-DDS transport. Higher-level text
     * logging must use a different channel, or disable micro-ROS first. */
    rmw_uros_set_custom_transport(
        true,
        (void *)uart,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    if (!mros_executor_configure_allocator(&context->allocator)) {
        return false;
    }

    do {
        rc = rclc_support_init(&context->support, 0, NULL, &context->allocator);
        if (rc != RCL_RET_OK) {
            osDelay(100);
        }
    } while (rc != RCL_RET_OK);

    rc = rclc_node_init_default(
        &context->node,
        mros_executor_resolve_name(node_name, MROS_EXECUTOR_DEFAULT_NODE_NAME),
        "",
        &context->support);
    if (rc != RCL_RET_OK) {
        return false;
    }

    context->initialized = true;
    return true;
}

/**
 * @brief executor handle pool을 준비합니다.
 * @details
 * - subscription callback을 붙이기 전에 executor가 몇 개의 handle을 담을지 미리 정해야 합니다.
 * - 이미 초기화된 경우에는 현재 capacity가 충분한지 검사만 수행합니다.
 * @param context executor 컨텍스트
 * @param handle_count 필요한 최대 handle 수
 * @retval 준비 성공 시 true
 */
bool mros_executor_prepare(
    mros_executor_context_t *context,
    size_t handle_count)
{
    if (context == NULL || !context->initialized) {
        return false;
    }

    if (handle_count == 0U) {
        return true;
    }

    if (context->executor_initialized) {
        return context->executor.max_handles >= handle_count;
    }

    if (rclc_executor_init(
            &context->executor,
            &context->support.context,
            handle_count,
            &context->allocator) != RCL_RET_OK) {
        return false;
    }

    context->executor_initialized = true;
    return true;
}

/**
 * @brief executor를 짧게 spin 하여 수신 callback을 처리합니다.
 * @param context executor 컨텍스트
 * @param timeout_ns 대기 시간(ns)
 * @retval 호출 성공 시 true
 */
bool mros_executor_spin_some(
    mros_executor_context_t *context,
    uint64_t timeout_ns)
{
    if (context == NULL || !context->executor_initialized) {
        return false;
    }

    return rclc_executor_spin_some(&context->executor, timeout_ns) == RCL_RET_OK;
}
