#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

// MPU 설정으로 캐시가 비활성화된 SRAM4 (Region 0, 0x38000000) 구역에 DMA 버퍼 배치
__attribute__((section(".dma_buffer"))) static uint8_t dma_rx_buffer[UART_DMA_BUFFER_SIZE];
__attribute__((section(".dma_buffer"))) static uint8_t dma_tx_buffer[UART_DMA_BUFFER_SIZE];

static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_Receive_DMA(uart, dma_rx_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY){
        // Non-cacheable 영역인 dma_tx_buffer로 데이터를 복사한 후 안전하게 DMA 송신
        size_t write_len = (len > UART_DMA_BUFFER_SIZE) ? UART_DMA_BUFFER_SIZE : len;
        memcpy(dma_tx_buffer, buf, write_len);
        
        ret = HAL_UART_Transmit_DMA(uart, dma_tx_buffer, write_len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
            osDelay(1);
        }

        return (ret == HAL_OK) ? write_len : 0;
    }else{
        return 0;
    }
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    int ms_used = 0;
    do
    {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();
        ms_used++;
        osDelay(portTICK_PERIOD_MS);
    } while (dma_head == dma_tail && ms_used < timeout);
    
    // dma_rx_buffer가 캐시 비활성화 영역(SRAM4)에 있으므로 수동 Invalidate 과정이 필요 없음
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_rx_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM