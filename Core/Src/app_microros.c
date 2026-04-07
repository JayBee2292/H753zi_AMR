#include "app_microros.h"

#include "cmsis_os2.h"

#include <string.h>

#include <rcl/error_handling.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

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

static bool app_microros_configure_allocator(void)
{
    rcl_allocator_t freertos_allocator = rcutils_get_zero_initialized_allocator();

    freertos_allocator.allocate = microros_allocate;
    freertos_allocator.deallocate = microros_deallocate;
    freertos_allocator.reallocate = microros_reallocate;
    freertos_allocator.zero_allocate = microros_zero_allocate;

    return rcutils_set_default_allocator(&freertos_allocator);
}

static const char *app_microros_resolve_name(const char *value, const char *fallback)
{
    return (value != NULL) ? value : fallback;
}

bool app_microros_init(
    app_microros_context_t *context,
    UART_HandleTypeDef *uart,
    const char *node_name,
    const char *topic_name)
{
    rcl_ret_t rc = RCL_RET_OK;

    if (context == NULL || uart == NULL) {
        return false;
    }

    memset(context, 0, sizeof(*context));
    context->node = rcl_get_zero_initialized_node();
    context->publisher = rcl_get_zero_initialized_publisher();

    rmw_uros_set_custom_transport(
        true,
        (void *)uart,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    if (!app_microros_configure_allocator()) {
        return false;
    }

    context->allocator = rcl_get_default_allocator();

    do {
        rc = rclc_support_init(&context->support, 0, NULL, &context->allocator);
        if (rc != RCL_RET_OK) {
            osDelay(100);
        }
    } while (rc != RCL_RET_OK);

    rc = rclc_node_init_default(
        &context->node,
        app_microros_resolve_name(node_name, APP_MICROROS_DEFAULT_NODE_NAME),
        "",
        &context->support);
    if (rc != RCL_RET_OK) {
        return false;
    }

    rc = rclc_publisher_init_default(
        &context->publisher,
        &context->node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        app_microros_resolve_name(topic_name, APP_MICROROS_DEFAULT_TWIST_TOPIC));
    if (rc != RCL_RET_OK) {
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

bool app_microros_publish_twist(
    app_microros_context_t *context,
    float linear_x,
    float angular_z)
{
    if (context == NULL || !context->initialized) {
        return false;
    }

    context->twist_msg.linear.x = linear_x;
    context->twist_msg.angular.z = angular_z;

    return rcl_publish(&context->publisher, &context->twist_msg, NULL) == RCL_RET_OK;
}
