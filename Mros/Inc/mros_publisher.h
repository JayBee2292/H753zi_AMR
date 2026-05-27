#ifndef MROS_PUBLISHER_H
#define MROS_PUBLISHER_H

#include <stdbool.h>

#include "mros_executor.h"

#include <geometry_msgs/msg/twist.h>

#define MROS_PUBLISHER_DEFAULT_TWIST_TOPIC "odom_vel" /* 기본 차체 속도 publish 토픽 이름 */

/**
 * @brief `geometry_msgs/Twist` 퍼블리셔 상태
 * @details
 * - 퍼블리셔 핸들과 재사용 가능한 메시지 버퍼를 함께 보관합니다.
 * - 매 publish마다 메시지 메모리를 다시 만들지 않도록 `twist_msg`를 구조체 내부에 유지합니다.
 */
typedef struct {
    rcl_publisher_t publisher;              /* ROS 2 publisher 핸들 */
    geometry_msgs__msg__Twist twist_msg;    /* 재사용할 Twist 메시지 인스턴스 */
    bool initialized;                       /* publisher 생성 완료 여부 */
} mros_publisher_context_t;

/**
 * @brief Twist 퍼블리셔를 생성합니다.
 * @param context 초기화할 publisher 컨텍스트
 * @param executor 이미 node 초기화된 executor 컨텍스트
 * @param topic_name publish할 토픽 이름. NULL이면 기본값 사용
 * @retval 성공 시 true
 */
bool mros_publisher_init_twist(
    mros_publisher_context_t *context,
    mros_executor_context_t *executor,
    const char *topic_name);

/**
 * @brief 로봇 선속도와 각속도를 Twist 메시지로 publish합니다.
 * @param context publisher 컨텍스트
 * @param linear_x x축 선속도(m/s)
 * @param angular_z z축 각속도(rad/s)
 * @retval publish 성공 시 true
 */
bool mros_publisher_publish_twist(
    mros_publisher_context_t *context,
    float linear_x,
    float angular_z);

#endif
