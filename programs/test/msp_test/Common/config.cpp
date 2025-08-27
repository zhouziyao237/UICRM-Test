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
// Created by Administrator on 2025/8/14.
//

#include "config.h"

#include "Device/bsp_Serial/DeviceSerial.h"
#include "Device/bsp_Serial/bsp_Serial.h"
#include "Device/config/feature.h"
#include "FreeRTOS.h"
#include "bsp_print.h"
#include "target.h"
#include "task.h"

#include "Module/RX/rx.h"

bool eepromWriteInProgress = false;
bool configIsDirty = false;

//
// 内部函数
//

static void validateAndFixConfig(void) {
    // 验证和修复配置
    // 在实际应用中可能需要根据具体配置项进行验证和修复
}

static void activateConfig(void) {
    // 激活配置
}

//
// EEPROM 函数
//

bool readEEPROM(void) {
    RMFC::RX::suspendRxSignal();

    // Sanity check, read flash
    bool success = loadEEPROM();
    featureInit();
    validateAndFixConfig();
    activateConfig();
    RMFC::RX::resumeRxSignal();

    return success;
}

void writeUnmodifiedConfigToEEPROM(void) {
    validateAndFixConfig();

#if defined(DEBUG_PG)
    print("Configuration before saving:\r\n");
    for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
        const RMFC::serialPortConfig_t* config = &RMFC::serialConfig()->portConfigs[i];
        print("Port %d: identifier=%d, functionMask=0x%04X\r\n", i, config->identifier,
              config->functionMask);
    }
#endif

    vTaskSuspendAll();  // 挂起所有任务以防止干扰
    eepromWriteInProgress = true;
    writeConfigToEEPROM();
    eepromWriteInProgress = false;
    xTaskResumeAll();  // 恢复所有任务

    configIsDirty = false;
}

void writeEEPROM(void) {
#ifdef USE_RX_SPI
    rxSpiStop();  // some rx spi protocols use hardware timer, which needs to be stopped before
                  // writing to eeprom
#endif
    //         systemConfigMutable()->configurationState = CONFIGURATION_STATE_CONFIGURED;

    writeUnmodifiedConfigToEEPROM();
}

bool resetEEPROM(bool useCustomDefaults) {
    if (!useCustomDefaults) {
        // resetConfig();
        pgResetAll();
    }

    writeUnmodifiedConfigToEEPROM();

    return true;
}

void ensureEEPROMStructureIsValid(void) {
    if (isEEPROMContentValid()) {
        return;
    }
    resetEEPROM(false);
}

void saveConfigAndNotify(void) {
    // The write to EEPROM will cause a big delay in the current task, so ignore
    vTaskSuspendAll();

    writeEEPROM();
    readEEPROM();

    xTaskResumeAll();

    // beeperConfirmationBeeps(1);
}

void setConfigDirty(void) {
    configIsDirty = true;
}

bool isConfigDirty(void) {
    return configIsDirty;
}
