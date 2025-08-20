// Copyright (c) 2025. BNU-HKBU UIC RoboMaster
//
// This program is free software: you can redistribute it
// and/or modify it under the terms of the GNU General
// Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at
// your option) any later version.
//
// This program is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even
// the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General
// Public License for more details.
//
// You should have received a copy of the GNU General
// Public License along with this program.  If not, see
// <https://www.gnu.org/licenses/>.

//
// Created by Administrator on 2025/8/10.
//

#ifndef TARGET_H
#define TARGET_H

//
// Uart User Config
//

#define USE_CUSTOM_DEFAULTS

#define USEUART
#define SERIAL_PORT_COUNT 6
#define MAX_MSP_PORT_COUNT 3

#define USE_DEBUGUART &huart8

#define USEUSB_VCP
#define USEUART_1 &huart1
#define USEUART_2 &huart2
#define USEUART_3 &huart3
#define USEUART_6 &huart6
#define USEUART_7 &huart7
#define MSP_FUNCTION_UART SERIAL_PORT_USART8
// #define USEUART_8 &huart8

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

//
// MPU User Config
//

#define ONBOARD_IMU_SPI hspi5
#define ONBOARD_IMU_CS_GROUP GPIOF
#define ONBOARD_IMU_CS_PIN GPIO_PIN_6

//
// Debug User Config
//

#define USE_CLI
#define DEBUG_PG
// #define DEBUG_FLASH
// #define DEBUG_MSP

#endif //TARGET_H
