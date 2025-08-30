/*
 * app_crc16.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Truong Quoc Huy
 */

#ifndef APP_INC_APP_CRC16_H_
#define APP_INC_APP_CRC16_H_

#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
uint16_t crc16_ccitt(const uint8_t *data, uint32_t len);
#ifdef __cplusplus
}
#endif


#endif /* APP_INC_APP_CRC16_H_ */
