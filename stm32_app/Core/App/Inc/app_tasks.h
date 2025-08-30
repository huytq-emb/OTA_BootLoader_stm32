/*
 * app_tasks.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#ifndef APP_INC_APP_TASKS_H_
#define APP_INC_APP_TASKS_H_

#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif

extern QueueHandle_t app_qh;

void app_tasks_preinit(void); // tạo queue dùng chung nếu cần

void blink_task(void *arg);
void logger_task(void *arg);
void producer_task(void *arg);
void consumer_task(void *arg);

#ifdef __cplusplus
}
#endif


#endif /* APP_INC_APP_TASKS_H_ */
