#include "mros_publisher.h"

/**
 * @brief NULL 토픽 이름을 기본 문자열로 치환합니다.
 * @param value 사용자 입력 문자열
 * @param fallback 기본 문자열
 * @retval 사용할 최종 토픽 이름
 */
static const char *mros_publisher_resolve_name(const char *value, const char *fallback)
{
    return (value != NULL) ? value : fallback;
}

/**
 * @brief `geometry_msgs/Twist` 퍼블리셔를 생성합니다.
 * @details
 * - 현재 프로젝트는 `linear.x`와 `angular.z`만 사용하지만,
 *   초기값을 전체 축에 0으로 명시해 예기치 않은 쓰레기 값이 publish되지 않게 합니다.
 * @param context 초기화할 퍼블리셔 컨텍스트
 * @param executor 이미 node 초기화가 끝난 executor 컨텍스트
 * @param topic_name 사용할 토픽 이름. NULL이면 기본 이름 사용
 * @retval 성공 시 true
 */
bool mros_publisher_init_twist(
    mros_publisher_context_t *context,
    mros_executor_context_t *executor,
    const char *topic_name)
{
    if (context == NULL || executor == NULL || !executor->initialized) {
        return false;
    }

    *context = (mros_publisher_context_t) {
        .publisher = rcl_get_zero_initialized_publisher(),
    };

    if (rclc_publisher_init_default(
            &context->publisher,
            &executor->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            mros_publisher_resolve_name(topic_name, MROS_PUBLISHER_DEFAULT_TWIST_TOPIC)) != RCL_RET_OK) {
        return false;
    }

    context->twist_msg.linear.x = 0.0;
    context->twist_msg.linear.y = 0.0;
    context->twist_msg.linear.z = 0.0;
    context->twist_msg.angular.x = 0.0;
    context->twist_msg.angular.y = 0.0;
    context->twist_msg.angular.z = 0.0;
    context->initialized = true;

    return true;
}

/**
 * @brief 현재 차체 속도를 Twist 메시지로 publish합니다.
 * @param context 퍼블리셔 컨텍스트
 * @param linear_x x축 선속도(m/s)
 * @param angular_z z축 각속도(rad/s)
 * @retval publish 성공 시 true
 */
bool mros_publisher_publish_twist(
    mros_publisher_context_t *context,
    float linear_x,
    float angular_z)
{
    if (context == NULL || !context->initialized) {
        return false;
    }

    context->twist_msg.linear.x = linear_x;
    context->twist_msg.angular.z = angular_z;

    /* Reuse the preallocated message to avoid per-cycle allocation churn. */
    return rcl_publish(&context->publisher, &context->twist_msg, NULL) == RCL_RET_OK;
}
