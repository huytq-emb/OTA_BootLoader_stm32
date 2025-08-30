/*
 * app_tasks.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#include "app.h"
#include "app_uart.h"
#include "app_tasks.h"

QueueHandle_t app_qh = NULL;

/**
 * @brief       Pre-initialize shared resources for tasks (queues, etc.)
 * @param       None
 * @retval      None
 */
void app_tasks_preinit(void) {
    app_qh = xQueueCreate(8, sizeof(uint32_t));
}

/**
 * @brief       LED blinking task
 * @param       arg Task parameter (unused)
 * @retval      None (never returns)
 */
void blink_task(void *arg) {
    (void)arg;
    for (;;) {
        LED_PORT->ODR ^= (1U << LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/**
 * @brief       Logging task that outputs system tick information
 * @param       arg Task parameter (unused)
 * @retval      None (never returns)
 */
void logger_task(void *arg) {
    (void)arg;
    for (;;) {
        const uint32_t tick = (uint32_t)xTaskGetTickCount();
        const unsigned hz   = (unsigned)configTICK_RATE_HZ;
        log_printf("[APP] tick=%lu tick_hz=%u\n", (unsigned long)tick, hz);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief       Producer task for queue demonstration
 * @param       arg Task parameter (unused)
 * @retval      None (never returns)
 */
void producer_task(void *arg) {
    (void)arg;
    uint32_t i = 0;
    for (;;) {
        if (app_qh) (void)xQueueSend(app_qh, &i, portMAX_DELAY);
        i++;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief       Consumer task for queue demonstration
 * @param       arg Task parameter (unused)
 * @retval      None (never returns)
 */
void consumer_task(void *arg) {
    (void)arg;
    uint32_t v;
    for (;;) {
        if (app_qh && xQueueReceive(app_qh, &v, portMAX_DELAY) == pdTRUE) {
            log_printf("[Q] got %lu\n", (unsigned long)v);
        }
    }
}

