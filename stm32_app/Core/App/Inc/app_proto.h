/*
 * app_proto.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#ifndef APP_INC_APP_PROTO_H_
#define APP_INC_APP_PROTO_H_

#pragma once
#include "FreeRTOS.h"
#include "task.h"
#ifdef __cplusplus
extern "C" {
#endif
void uart_cmd_task(void *arg);
#ifdef __cplusplus
}
#endif



#endif /* APP_INC_APP_PROTO_H_ */
