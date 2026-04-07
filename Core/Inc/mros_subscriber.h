#ifndef MROS_SUBSCRIBER_H
#define MROS_SUBSCRIBER_H

#include <stdbool.h>

#include "mros_executor.h"

#include <geometry_msgs/msg/twist.h>

#define MROS_SUBSCRIBER_DEFAULT_TWIST_TOPIC "cmd_vel" /* 기본 차체 속도 명령 구독 토픽 이름 */

/* 수신한 Twist 메시지를 사용자 컨텍스트와 함께 상위 로직으로 전달하는 콜백 형식 */
typedef void (*mros_subscriber_twist_callback_t)(
    const geometry_msgs__msg__Twist *msg,
    void *context);

/**
 * @brief `geometry_msgs/Twist` 구독 상태
 * @details
 * - subscription 핸들, 수신 버퍼, 사용자 콜백과 콜백 컨텍스트를 한 구조체에 보관합니다.
 * - `initialized`는 subscription 생성 완료, `attached`는 executor에 callback 연결 완료를 의미합니다.
 */
typedef struct {
    rcl_subscription_t subscription;                /* ROS 2 subscription 핸들 */
    geometry_msgs__msg__Twist twist_msg;           /* executor가 덮어쓸 수신 버퍼 */
    mros_subscriber_twist_callback_t callback;     /* 상위 로직에 전달할 사용자 콜백 */
    void *callback_context;                        /* 사용자 콜백에 함께 넘길 임의 컨텍스트 */
    bool initialized;                              /* subscription 생성 완료 여부 */
    bool attached;                                 /* executor에 add_subscription 완료 여부 */
} mros_subscriber_context_t;

/**
 * @brief Twist subscription을 생성합니다.
 * @param context 초기화할 subscriber 컨텍스트
 * @param executor 이미 node 초기화된 executor 컨텍스트
 * @param topic_name 구독할 토픽 이름. NULL이면 기본값 사용
 * @retval 성공 시 true
 */
bool mros_subscriber_init_twist(
    mros_subscriber_context_t *context,
    mros_executor_context_t *executor,
    const char *topic_name);

/**
 * @brief subscription을 executor에 연결하고 사용자 콜백을 등록합니다.
 * @param context subscriber 컨텍스트
 * @param executor executor 컨텍스트. `mros_executor_prepare()`가 먼저 호출되어야 합니다.
 * @param callback 메시지 수신 시 호출할 사용자 함수
 * @param callback_context 사용자 함수에 함께 전달할 컨텍스트 포인터
 * @retval 성공 시 true
 */
bool mros_subscriber_attach_twist(
    mros_subscriber_context_t *context,
    mros_executor_context_t *executor,
    mros_subscriber_twist_callback_t callback,
    void *callback_context);

#endif
