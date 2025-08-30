/*
 * app_system.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#ifndef APP_INC_APP_SYSTEM_H_
#define APP_INC_APP_SYSTEM_H_

#pragma once
#include "stm32f1xx.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void System_Init(void);
void relocate_vtor(void);

#ifdef __cplusplus
}
#endif


#endif /* APP_INC_APP_SYSTEM_H_ */
