#include "mros_subscriber.h"

/**
 * @brief NULL 토픽 이름을 기본 문자열로 치환합니다.
 * @param value 사용자 입력 문자열
 * @param fallback 기본 문자열
 * @retval 사용할 최종 토픽 이름
 */
static const char *mros_subscriber_resolve_name(const char *value, const char *fallback)
{
    return (value != NULL) ? value : fallback;
}

/**
 * @brief executor generic callback을 사용자 Twist 콜백으로 연결합니다.
 * @param msg executor가 전달한 수신 메시지 버퍼
 * @param context `mros_subscriber_context_t` 포인터
 */
static void mros_subscriber_twist_bridge(const void *msg, void *context)
{
    mros_subscriber_context_t *subscriber = (mros_subscriber_context_t *)context;

    if (subscriber == NULL || subscriber->callback == NULL || msg == NULL) {
        return;
    }

    subscriber->callback(
        (const geometry_msgs__msg__Twist *)msg,
        subscriber->callback_context);
}

/**
 * @brief `geometry_msgs/Twist` subscription을 생성합니다.
 * @param context 초기화할 subscriber 컨텍스트
 * @param executor 이미 node 초기화가 끝난 executor 컨텍스트
 * @param topic_name 구독할 토픽 이름. NULL이면 기본 이름 사용
 * @retval 성공 시 true
 */
bool mros_subscriber_init_twist(
    mros_subscriber_context_t *context,
    mros_executor_context_t *executor,
    const char *topic_name)
{
    if (context == NULL || executor == NULL || !executor->initialized) {
        return false;
    }

    *context = (mros_subscriber_context_t) {
        .subscription = rcl_get_zero_initialized_subscription(),
    };

    if (rclc_subscription_init_default(
            &context->subscription,
            &executor->node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            mros_subscriber_resolve_name(topic_name, MROS_SUBSCRIBER_DEFAULT_TWIST_TOPIC)) != RCL_RET_OK) {
        return false;
    }

    context->initialized = true;
    return true;
}

/**
 * @brief subscription을 executor에 연결하고 사용자 콜백을 등록합니다.
 * @details
 * - 이 단계가 끝나면 executor spin 시 `twist_msg`가 갱신되고 사용자 콜백이 호출됩니다.
 * - 호출 전 `mros_executor_prepare()`로 executor handle capacity를 확보해야 합니다.
 * @param context subscriber 컨텍스트
 * @param executor executor 컨텍스트
 * @param callback 새 메시지 수신 시 호출할 함수
 * @param callback_context 사용자 함수에 함께 넘길 임의 컨텍스트 포인터
 * @retval 성공 시 true
 */
bool mros_subscriber_attach_twist(
    mros_subscriber_context_t *context,
    mros_executor_context_t *executor,
    mros_subscriber_twist_callback_t callback,
    void *callback_context)
{
    if (context == NULL ||
        executor == NULL ||
        callback == NULL ||
        !context->initialized ||
        !executor->executor_initialized) {
        return false;
    }

    context->callback = callback;
    context->callback_context = callback_context;

    if (rclc_executor_add_subscription_with_context(
            &executor->executor,
            &context->subscription,
            &context->twist_msg,
            mros_subscriber_twist_bridge,
            context,
            ON_NEW_DATA) != RCL_RET_OK) {
        return false;
    }

    context->attached = true;
    return true;
}
