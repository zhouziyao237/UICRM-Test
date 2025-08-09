/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2022 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#ifndef _CRC_CHECK_H_
#define _CRC_CHECK_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

struct sbuf_s;

/**
 * Verify CRC8
 *
 * @param  pchMessage Message to verify
 * @param  dwLength   Length = Data + Checksum
 * @return            1 for true, 0 for false
 */
uint8_t verify_crc8_check_sum(const uint8_t* pchMessage, uint16_t dwLength);

/**
 * Append CRC8 to the end of message
 *
 * @param  pchMessage Message to calculate CRC and append
 * @param  dwLength   Length = Data + Checksum
 */
void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a);
uint8_t crc8_dvb_s2_update(uint8_t crc, const void* data, uint32_t length);
void crc8_dvb_s2_sbuf_append(struct sbuf_s* dst, uint8_t* start);
uint8_t crc8_xor(uint8_t crc, const uint8_t data);
uint8_t crc8_xor_update(uint8_t crc, const void* data, uint32_t length);
void crc8_xor_sbuf_append(struct sbuf_s* dst, uint8_t* start);

/**
 * @brief 计算数据的CRC16校验和
 *
 * @param pchMessage 指向待计算CRC校验的数据缓冲区
 * @param dwLength 数据缓冲区的长度（字节数）
 * @param wCRC 初始CRC值，通常传入0xFFFF或0x0000
 * @return 计算得到的CRC16校验和
 */
uint16_t get_crc16_check_sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);

/**
 * Verify CRC16
 *
 * @param  pchMessage Message to verify
 * @param  dwLength   Length = Data + Checksum
 * @return            1 for true, 0 for false
 */
uint8_t verify_crc16_check_sum(const uint8_t* pchMessage, uint32_t dwLength);

/**
 * Append CRC16 to the end of message
 *
 * @param  pchMessage Message to calculate CRC and append
 * @param  dwLength   Length = Data + Checksum
 */
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

uint16_t crc16_ccitt(uint16_t crc, unsigned char a);
uint16_t crc16_ccitt_update(uint16_t crc, const void* data, uint32_t length);
struct sbuf_s;
void crc16_ccitt_sbuf_append(struct sbuf_s* dst, uint8_t* start);

#ifdef __cplusplus
}
#endif

#endif
