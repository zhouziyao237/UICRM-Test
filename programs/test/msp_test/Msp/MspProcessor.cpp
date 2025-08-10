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
// Created by Administrator on 2025/8/9.
//

#include "MspProcessor.h"

#include "Msp.h"
#include "MspParser.h"
#include "bsp_serial.h"
#include "config/msp_protocol.h"
#include "crc_check.h"
#include "platform.h"
#include "utils.h"

// #include "Model/Model.h"
// #include "Hardware.h"

namespace RMFC {
    namespace Msp {
        bool MspProcessor::process(char c, MspMessage& msg, MspResponse& res, Serial& s) {
            _parser.parse(c, msg);

            if (msg.state == MSP_STATE_RECEIVED) {
                // debugMessage(msg, &s);
                switch (msg.dir) {
                    case MSP_TYPE_CMD:
                        processCommand(msg, res);
                        sendResponse(res, s);
                        msg = MspMessage();
                        res = MspResponse();
                        break;
                    case MSP_TYPE_REPLY:
                        // processCommand(msg, s);
                        break;
                }
                return true;
            }

            return msg.state != MSP_STATE_IDLE;
        }

        void MspProcessor::processCommand(MspMessage& m, MspResponse& r) {
            r.cmd = m.cmd;
            r.version = m.version;
            r.result = 1;
            switch (m.cmd) {
                case MSP_API_VERSION:
                    r.writeU8(MSP_PROTOCOL_VERSION);
                    r.writeU8(API_VERSION_MAJOR);
                    r.writeU8(API_VERSION_MINOR);
                    break;

                case MSP_FC_VARIANT:
                    r.writeData(flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
                    break;

                case MSP_FC_VERSION:
                    r.writeU8(FC_VERSION_MAJOR);
                    r.writeU8(FC_VERSION_MINOR);
                    r.writeU8(FC_VERSION_PATCH_LEVEL);
                    break;

                case MSP_BOARD_INFO:
                    r.writeData(boardIdentifier, BOARD_IDENTIFIER_LENGTH);
                    r.writeU16(
                        0);  // No other build targets currently have hardware revision detection.
                    r.writeU8(0);                   // 0 == FC
                    r.writeU8(0);                   // target capabilities
                    r.writeU8(strlen(targetName));  // target name
                    r.writeData(targetName, strlen(targetName));
                    r.writeU8(0);  // board name
                    r.writeU8(0);  // manufacturer name
                    for (size_t i = 0; i < 32; i++)
                        r.writeU8(0);  // signature
                    r.writeU8(255);    // mcu id: unknown
                    // 1.42
                    r.writeU8(2);  // configuration state: configured
                    // 1.43
                    // r.writeU16(_model.state.gyroPresent ? _model.state.gyroTimer.rate : 0);  //
                    // sample rate
                    // {
                    //   uint32_t problems = 0;
                    //   if(_model.state.accelBias.x == 0 && _model.state.accelBias.y == 0 &&
                    //   _model.state.accelBias.z == 0) {
                    //     problems |= 1 << 0; // acc calibration required
                    //   }
                    //   if(_model.config.output.protocol == ESC_PROTOCOL_DISABLED) {
                    //   problems |= 1 << 1; // no motor protocol
                    //   }
                    //   r.writeU32(problems); // configuration problems
                    // }
                    // 1.44
                    r.writeU8(0);  // spi dev count
                    r.writeU8(0);  // i2c dev count
                    break;

                case MSP_BUILD_INFO:
                    r.writeData(buildDate, BUILD_DATE_LENGTH);
                    r.writeData(buildTime, BUILD_TIME_LENGTH);
                    r.writeData(shortGitRevision, GIT_SHORT_REVISION_LENGTH);
                    break;

                case MSP_UID:
                    // r.writeU32(getBoardId0());
                    // r.writeU32(getBoardId1());
                    // r.writeU32(getBoardId2());
                    break;

                case MSP_STATUS_EX:

                case MSP_STATUS:
                    // r.writeU16(_model.state.loopTimer.delta);
                    // r.writeU16(_model.state.stats.loopTime());
                    // r.writeU16(_model.state.i2cErrorCount); // i2c error count
                    //         acc,     baro,    mag,     gps,     sonar,   gyro
                    // r.writeU16(_model.accelActive() | _model.baroActive() << 1 |
                    // _model.magActive()
                    // << 2 | 0 << 3 | 0 << 4 | _model.gyroActive() << 5);

                    // r.writeU16(_model.accelActive() | _model.baroActive() << 1 |
                    // _model.magActive()
                    // << 2 | 1 << 3 | 0 << 4 | _model.gyroActive() << 5);

                    // r.writeU32(_model.state.modeMask); // flight mode flags

                    r.writeU8(0);  // pid profile
                    // r.writeU16(lrintf(_model.state.stats.getCpuLoad()));
                    if (m.cmd == MSP_STATUS_EX) {
                        r.writeU8(1);  // max profile count
                        r.writeU8(0);  // current rate profile index
                    } else {           // MSP_STATUS
                        // r.writeU16(_model.state.gyroTimer.interval); // gyro cycle time
                        r.writeU16(0);
                    }

                    // flight mode flags (above 32 bits)
                    r.writeU8(0);  // count

                    // Write arming disable flags
                    //          r.writeU8(ARMING_DISABLED_FLAGS_COUNT);  // 1 byte, flag count
                    // r.writeU32(_model.state.armingDisabledFlags);  // 4 bytes, flags
                    r.writeU8(0);  // reboot required
                    break;

                case MSP_NAME:
                    // r.writeString(_model.config.modelName);

                    break;

                case MSP_SET_NAME:
                    // memset(&_model.config.modelName, 0, MODEL_NAME_LEN + 1);
                    // for(size_t i = 0; i < std::min((size_t)m.received, MODEL_NAME_LEN); i++)
                    // {
                    //   _model.config.modelName[i] = m.readU8();
                    // }
                    break;

                case MSP_BOXNAMES:
                    r.writeString("ARM;ANGLE;AIRMODE;BUZZER;FAILSAFE;");
                    break;

                case MSP_BOXIDS:
                    // r.writeU8(MODE_ARMED);
                    // r.writeU8(MODE_ANGLE);
                    // r.writeU8(MODE_AIRMODE);
                    // r.writeU8(MODE_BUZZER);
                    // r.writeU8(MODE_FAILSAFE);
                    break;

                case MSP_MODE_RANGES:
                    // for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
                    // {
                    //   r.writeU8(_model.config.conditions[i].id);
                    //   r.writeU8(_model.config.conditions[i].ch - AXIS_AUX_1);
                    //   r.writeU8((_model.config.conditions[i].min - 900) / 25);
                    //   r.writeU8((_model.config.conditions[i].max - 900) / 25);
                    // }
                    break;

                case MSP_MODE_RANGES_EXTRA:
                    // r.writeU8(ACTUATOR_CONDITIONS);
                    // for(size_t i = 0; i < ACTUATOR_CONDITIONS; i++)
                    // {
                    //   r.writeU8(_model.config.conditions[i].id);
                    //   r.writeU8(_model.config.conditions[i].logicMode);
                    //   r.writeU8(_model.config.conditions[i].linkId);
                    // }

                    break;

                case MSP_SET_MODE_RANGE:
                    // {
                    //   size_t i = m.readU8();
                    //   if(i < ACTUATOR_CONDITIONS)
                    //   {
                    //     _model.config.conditions[i].id = m.readU8();
                    //     _model.config.conditions[i].ch = m.readU8() + AXIS_AUX_1;
                    //     _model.config.conditions[i].min = m.readU8() * 25 + 900;
                    //     _model.config.conditions[i].max = m.readU8() * 25 + 900;
                    //     if(m.remain() >= 2) {
                    //       _model.config.conditions[i].logicMode = m.readU8(); // mode logic
                    //       _model.config.conditions[i].linkId = m.readU8(); // link to
                    //     }
                    //   }
                    //   else
                    //   {
                    //     r.result = -1;
                    //   }
                    // }
                    break;

                case MSP_ANALOG:
                    // r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage));  // voltage in
                    // 0.1V r.writeU16(0); // mah drawn r.writeU16(_model.getRssi()); // rssi
                    // r.writeU16(toIbatCurrent(_model.state.battery.current));  // amperage in
                    // 0.01A r.writeU16(toVbatVoltage(_model.state.battery.voltage));  // voltage in
                    // 0.01V r.writeU16(toVbatVoltage(101));  // voltage in 0.01V
                    break;

                case MSP_FEATURE_CONFIG:
                    // r.writeU32(_model.config.featureMask);
                    break;

                case MSP_SET_FEATURE_CONFIG:
                    // _model.config.featureMask = m.readU32();
                    // _model.reload();
                    break;

                case MSP_BATTERY_CONFIG:
                    r.writeU8(34);  // vbatmincellvoltage
                    r.writeU8(42);  // vbatmaxcellvoltage
                    // r.writeU8((_model.config.vbatCellWarning + 5) / 10);  //
                    // vbatwarningcellvoltage
                    r.writeU16(0);  // batteryCapacity
                    // r.writeU8(_model.config.vbatSource);  // voltageMeterSource
                    // r.writeU8(_model.config.ibatSource);  // currentMeterSource
                    r.writeU16(340);  // vbatmincellvoltage
                    r.writeU16(420);  // vbatmaxcellvoltage
                    // r.writeU16(_model.config.vbatCellWarning); // vbatwarningcellvoltage
                    break;

                case MSP_SET_BATTERY_CONFIG:
                    m.readU8();  // vbatmincellvoltage
                    m.readU8();  // vbatmaxcellvoltage
                    // _model.config.vbatCellWarning = m.readU8() * 10;  // vbatwarningcellvoltage
                    m.readU16();  // batteryCapacity
                    //          _model.config.vbatSource = toVbatSource(m.readU8());  //
                    //          voltageMeterSource _model.config.ibatSource =
                    //          toIbatSource(m.readU8());
                    //          // currentMeterSource
                    if (m.remain() >= 6) {
                        m.readU16();  // vbatmincellvoltage
                        m.readU16();  // vbatmaxcellvoltage
                        // _model.config.vbatCellWarning = m.readU16();
                    }
                    break;

                case MSP_BATTERY_STATE:
                    // battery characteristics
                    // r.writeU8(_model.state.battery.cells); // cell count, 0 indicates battery not
                    // detected.
                    r.writeU16(0);  // capacity in mAh

                    // battery state
                    // r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage)); // in 0.1V
                    // steps
                    r.writeU16(0);  // milliamp hours drawn from battery
                    // r.writeU16(toIbatCurrent(_model.state.battery.current)); // send current in
                    // 0.01 A steps, range is -320A to 320A

                    // battery alerts
                    r.writeU8(0);
                    // r.writeU16(toVbatVoltage(_model.state.battery.voltage)); // in 0.01 steps
                    break;

                case MSP_VOLTAGE_METERS:
                    for (int i = 0; i < 1; i++) {
                        r.writeU8(i + 10);  // meter id (10-19 vbat adc)
                        // r.writeU8(toVbatVoltageLegacy(_model.state.battery.voltage));  // meter
                        // value
                    }
                    break;

                case MSP_CURRENT_METERS:
                    for (int i = 0; i < 1; i++) {
                        r.writeU8(i + 10);  // meter id (10-19 ibat adc)
                        r.writeU16(0);      // mah drawn
                        // r.writeU16(constrain(toIbatCurrent(_model.state.battery.current) * 10, 0,
                        // 0xffff));  // meter value
                    }
                    break;

                case MSP_VOLTAGE_METER_CONFIG:
                    r.writeU8(1);  // num voltage sensors
                    for (int i = 0; i < 1; i++) {
                        r.writeU8(5);       // frame size (5)
                        r.writeU8(i + 10);  // id (10-19 vbat adc)
                        r.writeU8(0);       // type resistor divider
                        // r.writeU8(_model.config.vbatScale); // scale
                        // r.writeU8(_model.config.vbatResDiv);  // resdivval
                        // r.writeU8(_model.config.vbatResMult);  // resdivmultiplier
                    }
                    break;

                case MSP_SET_VOLTAGE_METER_CONFIG:
                    // {
                    //   int id = m.readU8();
                    //   if(id == 10 + 0) // id (10-19 vbat adc, allow only 10)
                    //   {
                    //     _model.config.vbatScale = m.readU8();
                    //     _model.config.vbatResDiv = m.readU8();
                    //     _model.config.vbatResMult = m.readU8();
                    //   }
                    // }
                    break;

                case MSP_CURRENT_METER_CONFIG:
                    r.writeU8(1);  // num voltage sensors
                    // for(int i = 0; i < 1; i++)
                    // {
                    //   r.writeU8(6); // frame size (6)
                    //   r.writeU8(i + 10); // id (10-19 ibat adc)
                    //   r.writeU8(1); // type adc
                    //   r.writeU16(_model.config.ibatScale); // scale
                    //   r.writeU16(_model.config.ibatOffset);  // offset
                    // }
                    break;

                case MSP_SET_CURRENT_METER_CONFIG:
                    // {
                    //   int id = m.readU8();
                    //   if(id == 10 + 0) // id (10-19 ibat adc, allow only 10)
                    //   {
                    //     _model.config.ibatScale = m.readU16();
                    //     _model.config.ibatOffset = m.readU16();
                    //   }
                    // }
                    break;

                case MSP_DATAFLASH_SUMMARY:
                    r.writeU8(0);  // FlashFS is neither ready nor supported
                    r.writeU32(0);
                    r.writeU32(0);
                    r.writeU32(0);
                    break;

                case MSP_ACC_TRIM:
                    r.writeU16(0);  // pitch
                    r.writeU16(0);  // roll
                    break;

                case MSP_MIXER_CONFIG:
                    // r.writeU8(_model.config.mixerType); // mixerMode, QUAD_X
                    // r.writeU8(_model.config.yawReverse); // yaw_motors_reversed
                    break;

                case MSP_SET_MIXER_CONFIG:
                    // _model.config.mixerType = m.readU8(); // mixerMode, QUAD_X
                    // _model.config.yawReverse = m.readU8(); // yaw_motors_reversed
                    break;

                case MSP_SENSOR_CONFIG:
                    // r.writeU8(_model.config.accelDev); // 3 acc mpu6050
                    // r.writeU8(_model.config.baroDev);  // 2 baro bmp085
                    // r.writeU8(_model.config.magDev);   // 3 mag hmc5883l
                    break;

                case MSP_SET_SENSOR_CONFIG:
                    // _model.config.accelDev = m.readU8(); // 3 acc mpu6050
                    // _model.config.baroDev = m.readU8();  // 2 baro bmp085
                    // _model.config.magDev = m.readU8();   // 3 mag hmc5883l
                    // _model.reload();
                    break;

                case MSP_SENSOR_ALIGNMENT:
                    // r.writeU8(_model.config.gyroAlign); // gyro align
                    // r.writeU8(_model.config.gyroAlign); // acc align, Starting with 4.0 gyro and
                    // acc alignment are the same r.writeU8(_model.config.magAlign);  // mag align
                    // //1.41+
                    // r.writeU8(_model.state.gyroPresent ? 1 : 0); // gyro detection mask
                    // GYRO_1_MASK r.writeU8(0); // gyro_to_use r.writeU8(_model.config.gyroAlign);
                    // // gyro 1 r.writeU8(0); // gyro 2
                    break;

                case MSP_SET_SENSOR_ALIGNMENT:
                    // {
                    //   uint8_t gyroAlign = m.readU8(); // gyro align
                    //   m.readU8(); // discard deprecated acc align
                    //   _model.config.magAlign = m.readU8(); // mag align
                    //   // API >= 1.41 - support the gyro_to_use and alignment for gyros 1 & 2
                    //   if(m.remain() >= 3)
                    //   {
                    //     m.readU8(); // gyro_to_use
                    //     gyroAlign = m.readU8(); // gyro 1 align
                    //     m.readU8(); // gyro 2 align
                    //   }
                    //   _model.config.gyroAlign = gyroAlign;
                    // }
                    break;

                case MSP_CF_SERIAL_CONFIG:
                    // for(int i = 0; i < SERIAL_UART_COUNT; i++)
                    // {
                    //   if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 &&
                    //   !_model.isActive(FEATURE_SOFTSERIAL)) continue;
                    //   r.writeU8(_model.config.serial[i].id); // identifier
                    //   r.writeU16(_model.config.serial[i].functionMask); // functionMask
                    //   r.writeU8(toBaudIndex(_model.config.serial[i].baud)); // msp_baudrateIndex
                    //   r.writeU8(0); // gps_baudrateIndex
                    //   r.writeU8(0); // telemetry_baudrateIndex
                    //   r.writeU8(toBaudIndex(_model.config.serial[i].blackboxBaud)); //
                    //   blackbox_baudrateIndex
                    // }
                    break;

                case MSP2_COMMON_SERIAL_CONFIG:
                    // {
                    //   uint8_t count = 0;
                    //   for (int i = 0; i < SERIAL_UART_COUNT; i++)
                    //   {
                    //     if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 &&
                    //     !_model.isActive(FEATURE_SOFTSERIAL)) continue; count++;
                    //   }
                    //   r.writeU8(count);
                    //   for (int i = 0; i < SERIAL_UART_COUNT; i++)
                    //   {
                    //     if(_model.config.serial[i].id >= SERIAL_ID_SOFTSERIAL_1 &&
                    //     !_model.isActive(FEATURE_SOFTSERIAL)) continue;
                    //     r.writeU8(_model.config.serial[i].id); // identifier
                    //     r.writeU32(_model.config.serial[i].functionMask); // functionMask
                    //     r.writeU8(toBaudIndex(_model.config.serial[i].baud)); //
                    //     msp_baudrateIndex r.writeU8(0); // gps_baudrateIndex r.writeU8(0); //
                    //     telemetry_baudrateIndex
                    //     r.writeU8(toBaudIndex(_model.config.serial[i].blackboxBaud)); //
                    //     blackbox_baudrateIndex
                    //   }
                    // }
                    break;

                case MSP_SET_CF_SERIAL_CONFIG:
                    // {
                    //   const int packetSize = 1 + 2 + 4;
                    //   while(m.remain() >= packetSize)
                    //   {
                    //     int id = m.readU8();
                    //     int k = _model.getSerialIndex((SerialPortId)id);
                    //     {
                    //       m.advance(packetSize - 1);
                    //       continue;
                    //     }
                    //     _model.config.serial[k].id = id;
                    //     _model.config.serial[k].functionMask = m.readU16();
                    //     _model.config.serial[k].baud =
                    //     fromBaudIndex((SerialSpeedIndex)m.readU8()); m.readU8(); m.readU8();
                    //     _model.config.serial[k].blackboxBaud =
                    //     fromBaudIndex((SerialSpeedIndex)m.readU8());
                    //   }
                    // }
                    // _model.reload();
                    break;

                case MSP2_COMMON_SET_SERIAL_CONFIG:
                    // {
                    //   size_t count = m.readU8();
                    //   (void)count; // ignore
                    //   const int packetSize = 1 + 4 + 4;
                    //   while(m.remain() >= packetSize)
                    //   {
                    //     int id = m.readU8();
                    //     int k = _model.getSerialIndex((SerialPortId)id);
                    //     if(k == -1)
                    //     {
                    //       m.advance(packetSize - 1);
                    //       continue;
                    //     }
                    //     _model.config.serial[k].id = id;
                    //     _model.config.serial[k].functionMask = m.readU32();
                    //     _model.config.serial[k].baud =
                    //     fromBaudIndex((SerialSpeedIndex)m.readU8()); m.readU8(); m.readU8();
                    //     _model.config.serial[k].blackboxBaud =
                    //     fromBaudIndex((SerialSpeedIndex)m.readU8());
                    //   }
                    // }
                    // _model.reload();
                    break;

                case MSP_BLACKBOX_CONFIG:
                    r.writeU8(1);  // Blackbox supported
                    // r.writeU8(_model.config.blackboxDev); // device serial or none
                    r.writeU8(1);  // blackboxGetRateNum()); // unused
                    r.writeU8(1);  // blackboxGetRateDenom());
                    // r.writeU16(_model.config.blackboxPdenom);//blackboxGetPRatio()); // p_denom
                    // r.writeU8(_model.config.blackboxPdenom); // sample_rate
                    // r.writeU32(_model.config.blackboxFieldsDisabledMask);
                    break;

                case MSP_SET_BLACKBOX_CONFIG:
                    // Don't allow config to be updated while Blackbox is logging
                    // if (true) {
                    //   _model.config.blackboxDev = m.readU8();
                    //   const int rateNum = m.readU8(); // was rate_num
                    //   const int rateDenom = m.readU8(); // was rate_denom
                    //   uint16_t pRatio = 0;
                    //   if (m.remain() >= 2) {
                    //       pRatio = m.readU16(); // p_denom specified, so use it directly
                    //   } else {
                    //       // p_denom not specified in MSP, so calculate it from old rateNum and
                    //       rateDenom
                    //       //pRatio = blackboxCalculatePDenom(rateNum, rateDenom);
                    //       (void)(rateNum + rateDenom);
                    //   }
                    //   _model.config.blackboxPdenom = pRatio;
                    //
                    //   /*if (m.remain() >= 1) {
                    //       _model.config.blackboxPdenom = m.readU8();
                    //   } else if(pRatio > 0) {
                    //       _model.config.blackboxPdenom = blackboxCalculateSampleRate(pRatio);
                    //       //_model.config.blackboxPdenom = pRatio;
                    //   }
                    //   if (m.remain() >= 4) {
                    //     _model.config.blackboxFieldsDisabledMask = m.readU32();
                    //   }*/
                    // }
                    break;

                case MSP_ATTITUDE:
                    // r.writeU16(lrintf(degrees(10) * 10.f)); // roll  [decidegrees]
                    // r.writeU16(lrintf(degrees(10) * 10.f)); // pitch [decidegrees]
                    // r.writeU16(lrintf(degrees(-10)));       // yaw   [degrees]

                    // r.writeU16(lrintf(degrees(_model.state.angle.x) * 10.f)); // roll
                    // [decidegrees] r.writeU16(lrintf(degrees(_model.state.angle.y) * 10.f)); //
                    // pitch [decidegrees] r.writeU16(lrintf(degrees(-_model.state.angle.z))); //
                    // yaw   [degrees]
                    break;

                case MSP_ALTITUDE:
                    // r.writeU32(lrintf(_model.state.baroAltitude * 100.f));    // alt [cm]
                    r.writeU16(0);  // vario
                    break;

                case MSP_BEEPER_CONFIG:
                    // r.writeU32(~_model.config.buzzer.beeperMask); // beeper mask
                    r.writeU8(0);   // dshot beacon tone
                    r.writeU32(0);  // dshot beacon off flags
                    break;

                case MSP_SET_BEEPER_CONFIG:
                    // _model.config.buzzer.beeperMask = ~m.readU32(); // beeper mask
                    break;

                case MSP_BOARD_ALIGNMENT_CONFIG:
                    r.writeU16(0);  // roll
                    r.writeU16(0);  // pitch
                    r.writeU16(0);  // yaw
                    break;

                case MSP_RX_MAP:
                    // for(size_t i = 0; i < INPUT_CHANNELS; i++)
                    // {
                    //   r.writeU8(_model.config.input.channel[i].map);
                    // }
                    break;

                case MSP_SET_RX_MAP:
                    // for(size_t i = 0; i < 8; i++)
                    // {
                    //   _model.config.input.channel[i].map = m.readU8();
                    // }
                    break;

                case MSP_RSSI_CONFIG:
                    // r.writeU8(_model.config.input.rssiChannel);
                    break;

                case MSP_SET_RSSI_CONFIG:
                    // _model.config.input.rssiChannel = m.readU8();
                    break;

                case MSP_MOTOR_CONFIG:
                    // r.writeU16(_model.config.output.minThrottle); // minthrottle
                    // r.writeU16(_model.config.output.maxThrottle); // maxthrottle
                    // r.writeU16(_model.config.output.minCommand);  // mincommand
                    // r.writeU8(_model.state.currentMixer.count);   // motor count
                    // 1.42+
                    r.writeU8(14);  // motor pole count
                    r.writeU8(0);   // dshot telemtery
                    r.writeU8(0);   // esc sensor
                    break;

                case MSP_SET_MOTOR_CONFIG:
                    // _model.config.output.minThrottle = m.readU16(); // minthrottle
                    // _model.config.output.maxThrottle = m.readU16(); // maxthrottle
                    // _model.config.output.minCommand = m.readU16();  // mincommand
                    if (m.remain() >= 2) {
                        m.readU8();
                        m.readU8();
                    }
                    // _model.reload();
                    break;

                case MSP_MOTOR_3D_CONFIG:
                    r.writeU16(1406);  // deadband3d_low;
                    r.writeU16(1514);  // deadband3d_high;
                    r.writeU16(1460);  // neutral3d;
                    break;

                case MSP_ARMING_CONFIG:
                    r.writeU8(5);    // auto_disarm delay
                    r.writeU8(0);    // disarm kill switch
                    r.writeU8(180);  // small angle
                    break;

                case MSP_RC_DEADBAND:
                    // r.writeU8(_model.config.input.deadband);
                    r.writeU8(0);   // yaw deadband
                    r.writeU8(0);   // alt hold deadband
                    r.writeU16(0);  // deadband 3d throttle
                    break;

                case MSP_SET_RC_DEADBAND:
                    // _model.config.input.deadband = m.readU8();
                    m.readU8();   // yaw deadband
                    m.readU8();   // alt hod deadband
                    m.readU16();  // deadband 3d throttle
                    break;

                case MSP_RX_CONFIG:
                    // r.writeU8(_model.config.input.serialRxProvider); // serialrx_provider
                    // r.writeU16(_model.config.input.maxCheck); //maxcheck
                    // r.writeU16(_model.config.input.midRc); //midrc
                    // r.writeU16(_model.config.input.minCheck); //mincheck
                    r.writeU8(0);  // spectrum bind
                    // r.writeU16(_model.config.input.minRc); //min_us
                    // r.writeU16(_model.config.input.maxRc); //max_us
                    // r.writeU8(_model.config.input.interpolationMode); // rc interpolation
                    // r.writeU8(_model.config.input.interpolationInterval); // rc interpolation
                    // interval
                    r.writeU16(1500);  // airmode activate threshold
                    r.writeU8(0);      // rx spi prot
                    r.writeU32(0);     // rx spi id
                    r.writeU8(0);      // rx spi chan count
                    r.writeU8(0);      // fpv camera angle
                    r.writeU8(2);      // rc iterpolation channels: RPYT
                    // r.writeU8(_model.config.input.filterType); // rc_smoothing_type
                    // r.writeU8(_model.config.input.filter.freq); // rc_smoothing_input_cutoff
                    // r.writeU8(_model.config.input.filterDerivative.freq); //
                    // rc_smoothing_derivative_cutoff
                    r.writeU8(0);  //_model.config.input.filter.type); // rc_smoothing_input_type
                    // r.writeU8(fromFilterTypeDerivative(_model.config.input.filterDerivative.type));
                    // // rc_smoothing_derivative_type
                    r.writeU8(0);  // usb type
                    // 1.42+
                    // r.writeU8(_model.config.input.filterAutoFactor); // rc_smoothing_auto_factor
                    break;

                case MSP_SET_RX_CONFIG:
                    // _model.config.input.serialRxProvider = m.readU8(); // serialrx_provider
                    // _model.config.input.maxCheck = m.readU16(); //maxcheck
                    // _model.config.input.midRc = m.readU16(); //midrc
                    // _model.config.input.minCheck = m.readU16(); //mincheck
                    // m.readU8(); // spectrum bind
                    // _model.config.input.minRc = m.readU16(); //min_us
                    // _model.config.input.maxRc = m.readU16(); //max_us
                    // if (m.remain() >= 4) {
                    //   _model.config.input.interpolationMode = m.readU8(); // rc interpolation
                    //   _model.config.input.interpolationInterval = m.readU8(); // rc interpolation
                    //   interval m.readU16(); // airmode activate threshold
                    // }
                    // if (m.remain() >= 6) {
                    //   m.readU8(); // rx spi prot
                    //   m.readU32(); // rx spi id
                    //   m.readU8(); // rx spi chan count
                    // }
                    // if (m.remain() >= 1) {
                    //   m.readU8(); // fpv camera angle
                    // }
                    // // 1.40+
                    // if (m.remain() >= 6) {
                    //   m.readU8(); // rc iterpolation channels
                    //   _model.config.input.filterType = m.readU8(); // rc_smoothing_type
                    //   _model.config.input.filter.freq = m.readU8(); // rc_smoothing_input_cutoff
                    //   _model.config.input.filterDerivative.freq = m.readU8(); //
                    //   rc_smoothing_derivative_cutoff
                    //   //_model.config.input.filter.type = m.readU8() == 1 ? FILTER_BIQUAD :
                    //   FILTER_PT1; // rc_smoothing_input_type m.readU8();
                    //   _model.config.input.filterDerivative.type =
                    //   toFilterTypeDerivative(m.readU8());
                    //   // rc_smoothing_derivative_type
                    // }
                    // if (m.remain() >= 1) {
                    //   m.readU8(); // usb type
                    // }
                    // // 1.42+
                    // if (m.remain() >= 1) {
                    //   _model.config.input.filterAutoFactor = m.readU8(); //
                    //   rc_smoothing_auto_factor
                    // }
                    //
                    // _model.reload();
                    break;

                case MSP_FAILSAFE_CONFIG:
                    // r.writeU8(_model.config.failsafe.delay); // failsafe_delay
                    r.writeU8(0);      // failsafe_off_delay
                    r.writeU16(1000);  // failsafe_throttle
                    // r.writeU8(_model.config.failsafe.killSwitch); // failsafe_kill_switch
                    r.writeU16(0);  // failsafe_throttle_low_delay
                    r.writeU8(1);   // failsafe_procedure; default drop
                    break;

                case MSP_SET_FAILSAFE_CONFIG:
                    // _model.config.failsafe.delay = m.readU8(); //failsafe_delay
                    m.readU8();   // failsafe_off_delay
                    m.readU16();  // failsafe_throttle
                    // _model.config.failsafe.killSwitch = m.readU8(); //failsafe_kill_switch
                    m.readU16();  // failsafe_throttle_low_delay
                    m.readU8();   // failsafe_procedure
                    break;

                case MSP_RXFAIL_CONFIG:
                    // for (size_t i = 0; i < _model.state.inputChannelCount; i++)
                    // {
                    //   r.writeU8(_model.config.input.channel[i].fsMode);
                    //   r.writeU16(_model.config.input.channel[i].fsValue);
                    // }
                    break;

                case MSP_SET_RXFAIL_CONFIG:
                    // {
                    //   size_t i = m.readU8();
                    //   if(i < INPUT_CHANNELS)
                    //   {
                    //     _model.config.input.channel[i].fsMode = m.readU8(); // mode
                    //     _model.config.input.channel[i].fsValue = m.readU16(); // pulse
                    //   }
                    //   else
                    //   {
                    //     r.result = -1;
                    //   }
                    // }
                    break;

                case MSP_RC:
                    // for(size_t i = 0; i < _model.state.inputChannelCount; i++)
                    // {
                    //   r.writeU16(lrintf(_model.state.inputUs[i]));
                    // }
                    break;

                case MSP_RC_TUNING:
                    // r.writeU8(_model.config.input.rate[AXIS_ROLL]);
                    // r.writeU8(_model.config.input.expo[AXIS_ROLL]);
                    // for(size_t i = 0; i < 3; i++)
                    // {
                    //   r.writeU8(_model.config.input.superRate[i]);
                    // }
                    // r.writeU8(_model.config.tpaScale); // dyn thr pid
                    // r.writeU8(50); // thrMid8
                    // r.writeU8(0);  // thr expo
                    // r.writeU16(_model.config.tpaBreakpoint); // tpa breakpoint
                    // r.writeU8(_model.config.input.expo[AXIS_YAW]); // yaw expo
                    // r.writeU8(_model.config.input.rate[AXIS_YAW]); // yaw rate
                    // r.writeU8(_model.config.input.rate[AXIS_PITCH]); // pitch rate
                    // r.writeU8(_model.config.input.expo[AXIS_PITCH]); // pitch expo
                    // // 1.41+
                    // r.writeU8(_model.config.output.throttleLimitType); // throttle_limit_type
                    // (off) r.writeU8(_model.config.output.throttleLimitPercent); //
                    // throtle_limit_percent (100%)
                    // //1.42+
                    // r.writeU16(_model.config.input.rateLimit[0]); // rate limit roll
                    // r.writeU16(_model.config.input.rateLimit[1]); // rate limit pitch
                    // r.writeU16(_model.config.input.rateLimit[2]); // rate limit yaw
                    // // 1.43+
                    // r.writeU8(_model.config.input.rateType); // rates type

                    break;

                case MSP_SET_RC_TUNING:
                    // if(m.remain() >= 10)
                    // {
                    //   const uint8_t rate = m.readU8();
                    //   if(_model.config.input.rate[AXIS_PITCH] ==
                    //   _model.config.input.rate[AXIS_ROLL])
                    //   {
                    //     _model.config.input.rate[AXIS_PITCH] = rate;
                    //   }
                    //   _model.config.input.rate[AXIS_ROLL] = rate;
                    //
                    //   const uint8_t expo = m.readU8();
                    //   if(_model.config.input.expo[AXIS_PITCH] ==
                    //   _model.config.input.expo[AXIS_ROLL])
                    //   {
                    //     _model.config.input.expo[AXIS_PITCH] = expo;
                    //   }
                    //   _model.config.input.expo[AXIS_ROLL] = expo;
                    //
                    //   for(size_t i = 0; i < 3; i++)
                    //   {
                    //     _model.config.input.superRate[i] = m.readU8();
                    //   }
                    //   _model.config.tpaScale = Math::clamp(m.readU8(), (uint8_t)0, (uint8_t)90);
                    //   // dyn thr pid m.readU8(); // thrMid8 m.readU8();  // thr expo
                    //   _model.config.tpaBreakpoint = Math::clamp(m.readU16(), (uint16_t)1000,
                    //   (uint16_t)2000); // tpa breakpoint if(m.remain() >= 1)
                    //   {
                    //     _model.config.input.expo[AXIS_YAW] = m.readU8(); // yaw expo
                    //   }
                    //   if(m.remain() >= 1)
                    //   {
                    //     _model.config.input.rate[AXIS_YAW]  = m.readU8(); // yaw rate
                    //   }
                    //   if(m.remain() >= 1)
                    //   {
                    //     _model.config.input.rate[AXIS_PITCH] = m.readU8(); // pitch rate
                    //   }
                    //   if(m.remain() >= 1)
                    //   {
                    //     _model.config.input.expo[AXIS_PITCH]  = m.readU8(); // pitch expo
                    //   }
                    //   // 1.41
                    //   if(m.remain() >= 2)
                    //   {
                    //     _model.config.output.throttleLimitType = m.readU8(); //
                    //     throttle_limit_type _model.config.output.throttleLimitPercent =
                    //     m.readU8(); // throttle_limit_percent
                    //   }
                    //   // 1.42
                    //   if(m.remain() >= 6)
                    //   {
                    //     _model.config.input.rateLimit[0] = m.readU16(); // roll
                    //     _model.config.input.rateLimit[1] = m.readU16(); // pitch
                    //     _model.config.input.rateLimit[2] = m.readU16(); // yaw
                    //   }
                    //   // 1.43
                    //   if (m.remain() >= 1)
                    //   {
                    //     _model.config.input.rateType = m.readU8();
                    //   }
                    // }
                    // else
                    // {
                    //   r.result = -1;
                    //   // error
                    // }
                    break;

                case MSP_ADVANCED_CONFIG:
                    // r.writeU8(1); // gyroSync unused
                    // r.writeU8(_model.config.loopSync);
                    // r.writeU8(_model.config.output.async);
                    // r.writeU8(_model.config.output.protocol);
                    // r.writeU16(_model.config.output.rate);
                    // r.writeU16(_model.config.output.dshotIdle);
                    // r.writeU8(0);    // 32k gyro
                    // r.writeU8(0);    // PWM inversion
                    // r.writeU8(0);    // gyro_to_use: {1:0, 2:1. 2:both}
                    // r.writeU8(0);    // gyro high fsr (flase)
                    // r.writeU8(48);   // gyro cal threshold
                    // r.writeU16(125); // gyro cal duration (1.25s)
                    // r.writeU16(0);   // gyro offset yaw
                    // r.writeU8(0);    // check overflow
                    // r.writeU8(_model.config.debugMode);
                    // r.writeU8(DEBUG_COUNT);
                    break;

                case MSP_SET_ADVANCED_CONFIG:
                    // m.readU8(); // ignore gyroSync, removed in 1.43
                    // _model.config.loopSync = m.readU8();
                    // _model.config.output.async = m.readU8();
                    // _model.config.output.protocol = m.readU8();
                    // _model.config.output.rate = m.readU16();
                    // if(m.remain() >= 2) {
                    //   _model.config.output.dshotIdle = m.readU16(); // dshot idle
                    // }
                    // if(m.remain()) {
                    //   m.readU8();  // 32k gyro
                    // }
                    // if(m.remain()) {
                    //   m.readU8();  // PWM inversion
                    // }
                    // if(m.remain() >= 8) {
                    //   m.readU8();  // gyro_to_use
                    //   m.readU8();  // gyro high fsr
                    //   m.readU8();  // gyro cal threshold
                    //   m.readU16(); // gyro cal duration
                    //   m.readU16(); // gyro offset yaw
                    //   m.readU8();  // check overflow
                    // }
                    // if(m.remain()) {
                    //   _model.config.debugMode = m.readU8();
                    // }
                    // _model.reload();
                    break;

                case MSP_GPS_CONFIG:
                    r.writeU8(0);  // provider
                    r.writeU8(0);  // sbasMode
                    r.writeU8(0);  // autoConfig
                    r.writeU8(0);  // autoBaud
                    // 1.43+
                    r.writeU8(0);  // gps_set_home_point_once
                    r.writeU8(0);  // gps_ublox_use_galileo

                    // r.writeU8(_model.config.GPSconfig.provider_GPS); // provider
                    // r.writeU8(_model.config.GPSconfig.sbasMode_GPS); // sbasMode
                    // r.writeU8(_model.config.GPSconfig.autoConfig_GPS); // autoConfig
                    // r.writeU8(_model.config.GPSconfig.autoBaud_GPS); // autoBaud
                    // // 1.43+
                    // r.writeU8(_model.config.GPSconfig.gps_set_home_point_once_GPS); //
                    // gps_set_home_point_once
                    // r.writeU8(_model.config.GPSconfig.gps_ublox_use_galileo_GPS); //
                    // gps_ublox_use_galileo
                    break;

                case MSP_SET_GPS_CONFIG:
                    // if(m.remain() >= 6 ){
                    // _model.config.GPSconfig.provider_GPS = m.readU8();
                    // _model.config.GPSconfig.sbasMode_GPS = m.readU8();
                    // _model.config.GPSconfig.autoConfig_GPS = m.readU8();
                    // _model.config.GPSconfig.autoBaud_GPS = m.readU8();
                    // _model.config.GPSconfig.gps_set_home_point_once_GPS = m.readU8();
                    // _model.config.GPSconfig.gps_ublox_use_galileo_GPS = m.readU8();
                    // }
                    // _model.reload();
                    break;

                case MSP_RAW_GPS:
                    r.writeU8(1);   // 写入GPS固定状态（0表示没有固定，1表示固定）
                    r.writeU8(22);  // gpsSol.numSat 写入当前可见的卫星数量
                    r.writeU32(
                        33 *
                        1e7);  // dst, gpsSol.llh.lat 写入纬度值，以度为单位，乘以1e7（百万分之一）
                    r.writeU32(
                        44 * 1e7);  // gpsSol.llh.lon 写入经度值，以度为单位，乘以1e7（百万分之一）
                    r.writeU16((uint16_t)constrain(1000 / 100, 0,
                                                   UINT16_MAX));  // gpsSol.llh.altCm
                    // 入高度值，原始数据是厘米，这里转换为米，并且每0.01米为一个单位，为了保持向后兼容性，这里将数据转换回每米一个单位
                    r.writeU16(66);  // gpsSol.groundSpeed 写入地面速度，单位为厘米/秒
                    r.writeU16(
                        1000 *
                        10);  // gpsSol.groundCourse 写入地面航向，单位为度，乘以10（十分之一）
                    r.writeU16(100);  // gpsSol.dop.pdop 写入位置精度（DOP），单位为厘米
                    break;

                case MSP_COMP_GPS:
                    r.writeU16(1);  // GPS_distanceToHome 处理MSP_COMP_GPS命令，发送计算后的GPS数据
                    // 写入到家的距离，单位为厘米
                    r.writeU16(1 / 10);  // GPS_directionToHome / 10
                    // 写入到家的方向，单位为度，Betaflight 4.4将分辨率提高了10倍，这里除以10以保持向后兼容性
                    r.writeU8(11 & 1);  // GPS_update 写入GPS更新标志
                    break;

                case MSP_GPSSVINFO:
                    r.writeU8(11);  // GPS_numCh 处理MSP_GPSSVINFO命令，发送GPS卫星信息
                    for (int i = 0; i < 11; i++) {
                        r.writeU8(1);  // GPS_svinfo_chn[i] 写入每个卫星的信道编号
                        r.writeU8(1);  // GPS_svinfo_svid[i] 写入每个卫星的ID
                        r.writeU8(1);  // GPS_svinfo_quality[i] 写入每个卫星的信号质量
                        r.writeU8(1);  // GPS_svinfo_cno[i] 写入每个卫星的信噪比

                        // r.writeU8(_model.config.GPSconfig.provider_GPS); // provider
                        // r.writeU8(_model.config.GPSconfig.sbasMode_GPS); // sbasMode
                        // r.writeU8(_model.config.GPSconfig.autoConfig_GPS); // autoConfig
                        // r.writeU8(_model.config.GPSconfig.autoBaud_GPS); // autoBaud
                    }
                    break;

                    // betafilght 固件4.5GPSAPI

                case MSP_FILTER_CONFIG:
                    // r.writeU8(_model.config.gyroFilter.freq);           // gyro lpf
                    // r.writeU16(_model.config.dtermFilter.freq);         // dterm lpf
                    // r.writeU16(_model.config.yawFilter.freq);           // yaw lpf
                    // r.writeU16(_model.config.gyroNotch1Filter.freq);    // gyro notch 1 hz
                    // r.writeU16(_model.config.gyroNotch1Filter.cutoff);  // gyro notch 1 cutoff
                    // r.writeU16(_model.config.dtermNotchFilter.freq);    // dterm notch hz
                    // r.writeU16(_model.config.dtermNotchFilter.cutoff);  // dterm notch cutoff
                    // r.writeU16(_model.config.gyroNotch2Filter.freq);    // gyro notch 2 hz
                    // r.writeU16(_model.config.gyroNotch2Filter.cutoff);  // gyro notch 2 cutoff
                    // r.writeU8(_model.config.dtermFilter.type);          // dterm type
                    // // r.writeU8(fromGyroDlpf(_model.config.gyroDlpf));
                    // r.writeU8(0);                                       // dlfp 32khz type
                    // r.writeU16(_model.config.gyroFilter.freq);          // lowpass1 freq
                    // r.writeU16(_model.config.gyroFilter2.freq);         // lowpass2 freq
                    // r.writeU8(_model.config.gyroFilter.type);           // lowpass1 type
                    // r.writeU8(_model.config.gyroFilter2.type);          // lowpass2 type
                    // r.writeU16(_model.config.dtermFilter2.freq);        // dterm lopwass2 freq
                    // // 1.41+
                    // r.writeU8(_model.config.dtermFilter2.type);         // dterm lopwass2 type
                    // r.writeU16(_model.config.gyroDynLpfFilter.cutoff);  // dyn lpf gyro min
                    // r.writeU16(_model.config.gyroDynLpfFilter.freq);    // dyn lpf gyro max
                    // r.writeU16(_model.config.dtermDynLpfFilter.cutoff); // dyn lpf dterm min
                    // r.writeU16(_model.config.dtermDynLpfFilter.freq);   // dyn lpf dterm max
                    // // gyro analyse
                    // r.writeU8(3);  // deprecated dyn notch range
                    // r.writeU8(_model.config.dynamicFilter.width);  // dyn_notch_width_percent
                    // r.writeU16(_model.config.dynamicFilter.q); // dyn_notch_q
                    // r.writeU16(_model.config.dynamicFilter.min_freq); // dyn_notch_min_hz
                    // // rpm filter
                    // r.writeU8(0);  // gyro_rpm_notch_harmonics
                    // r.writeU8(0);  // gyro_rpm_notch_min
                    // // 1.43+
                    // r.writeU16(_model.config.dynamicFilter.max_freq); // dyn_notch_max_hz

                    break;

                case MSP_SET_FILTER_CONFIG:
                    //          _model.config.gyroFilter.freq = m.readU8();
                    //          _model.config.dtermFilter.freq = m.readU16();
                    //          _model.config.yawFilter.freq = m.readU16();
                    //          if (m.remain() >= 8) {
                    //              _model.config.gyroNotch1Filter.freq = m.readU16();
                    //              _model.config.gyroNotch1Filter.cutoff = m.readU16();
                    //              _model.config.dtermNotchFilter.freq = m.readU16();
                    //              _model.config.dtermNotchFilter.cutoff = m.readU16();
                    //          }
                    //          if (m.remain() >= 4) {
                    //              _model.config.gyroNotch2Filter.freq = m.readU16();
                    //              _model.config.gyroNotch2Filter.cutoff = m.readU16();
                    //          }
                    //          if (m.remain() >= 1) {
                    //              _model.config.dtermFilter.type = (FilterType)m.readU8();
                    //          }
                    //          if (m.remain() >= 10) {
                    //            m.readU8(); // dlfp type
                    //            m.readU8(); // 32k dlfp type
                    //            _model.config.gyroFilter.freq = m.readU16();
                    //            _model.config.gyroFilter2.freq = m.readU16();
                    //            _model.config.gyroFilter.type = m.readU8();
                    //            _model.config.gyroFilter2.type = m.readU8();
                    //            _model.config.dtermFilter2.freq = m.readU16();
                    //          }
                    //          // 1.41+
                    //          if (m.remain() >= 9) {
                    //            _model.config.dtermFilter2.type = m.readU8();
                    //            _model.config.gyroDynLpfFilter.cutoff = m.readU16();  // dyn gyro
                    //            lpf min _model.config.gyroDynLpfFilter.freq = m.readU16();    //
                    //            dyn gyro lpf max _model.config.dtermDynLpfFilter.cutoff =
                    //            m.readU16(); // dyn dterm lpf min
                    //            _model.config.dtermDynLpfFilter.freq = m.readU16();   // dyn dterm
                    //            lpf min
                    //          }
                    //          if (m.remain() >= 8) {
                    //            m.readU8();  // deprecated dyn_notch_range
                    //            _model.config.dynamicFilter.width = m.readU8();  //
                    //            dyn_notch_width_percent _model.config.dynamicFilter.q =
                    //            m.readU16();
                    //            // dyn_notch_q _model.config.dynamicFilter.min_freq = m.readU16();
                    //            // dyn_notch_min_hz m.readU8();  // gyro_rpm_notch_harmonics
                    //            m.readU8();
                    //            // gyro_rpm_notch_min
                    //          }
                    //          // 1.43+
                    //          if (m.remain() >= 1) {
                    //            _model.config.dynamicFilter.max_freq = m.readU16(); //
                    //            dyn_notch_max_hz
                    //          }
                    //          _model.reload();
                    break;

                case MSP_PID_CONTROLLER:
                    r.writeU8(1);  // betaflight controller id
                    break;

                case MSP_PIDNAMES:
                    r.writeString("ROLL;PITCH;YAW;ALT;Pos;PosR;NavR;LEVEL;MAG;VEL;");
                    break;

                case MSP_PID:
                    // for(size_t i = 0; i < 2; i++)
                    // {
                    //   r.writeU8(_model.config.pid[i].P);
                    //   r.writeU8(_model.config.pid[i].I);
                    //   r.writeU8(_model.config.pid[i].D);
                    // }
                    break;

                case MSP_SET_PID:
                    // for (int i = 0; i < 2; i++)
                    // {
                    //   _model.config.pid[i].P = m.readU8();
                    //   _model.config.pid[i].I = m.readU8();
                    //   _model.config.pid[i].D = m.readU8();
                    // }
                    // _model.reload();
                    break;

                case MSP_PID_ADVANCED:  /// !!!FINISHED HERE!!!
                    r.writeU16(0);
                    r.writeU16(0);
                    r.writeU16(0);  // was pidProfile.yaw_p_limit
                    r.writeU8(0);   // reserved
                    r.writeU8(0);   // vbatPidCompensation;
                    r.writeU8(0);   // feedForwardTransition;
                    // r.writeU8((uint8_t)std::min(_model.config.dtermSetpointWeight,
                    // (int16_t)255)); // was low byte of dtermSetpointWeight
                    r.writeU8(0);   // reserved
                    r.writeU8(0);   // reserved
                    r.writeU8(0);   // reserved
                    r.writeU16(0);  // rateAccelLimit;
                    r.writeU16(0);  // yawRateAccelLimit;
                    // r.writeU8(_model.config.angleLimit); // levelAngleLimit;
                    r.writeU8(0);      // was pidProfile.levelSensitivity
                    r.writeU16(0);     // itermThrottleThreshold;
                    r.writeU16(1000);  // itermAcceleratorGain; anti_gravity_gain, 0 in 1.45+
                    // r.writeU16(_model.config.dtermSetpointWeight);
                    r.writeU8(0);  // iterm rotation
                    r.writeU8(0);  // smart feed forward
                    // r.writeU8(_model.config.itermRelax); // iterm relax
                    r.writeU8(1);  // iterm relax type (setpoint only)
                    r.writeU8(0);  // abs control gain
                    r.writeU8(0);  // throttle boost
                    r.writeU8(0);  // acro trainer max angle
                    //          r.writeU16(_model.config.pid[PID_ROLL].F); //pid roll f
                    //          r.writeU16(_model.config.pid[PID_PITCH].F); //pid pitch f
                    //          r.writeU16(_model.config.pid[PID_YAW].F); //pid yaw f
                    r.writeU8(0);  // antigravity mode
                    // 1.41+
                    r.writeU8(0);  // d min roll
                    r.writeU8(0);  // d min pitch
                    r.writeU8(0);  // d min yaw
                    r.writeU8(0);  // d min gain
                    r.writeU8(0);  // d min advance
                    r.writeU8(0);  // use_integrated_yaw
                    r.writeU8(0);  // integrated_yaw_relax
                    // 1.42+
                    // r.writeU8(_model.config.itermRelaxCutoff); // iterm_relax_cutoff
                    // // 1.43+
                    // r.writeU8(_model.config.output.motorLimit); // motor_output_limit
                    r.writeU8(0);  // auto_profile_cell_count
                    r.writeU8(0);  // idle_min_rpm

                    break;

                case MSP_SET_PID_ADVANCED:
                    // m.readU16();
                    // m.readU16();
                    // m.readU16(); // was pidProfile.yaw_p_limit
                    // m.readU8(); // reserved
                    // m.readU8();
                    // m.readU8();
                    // _model.config.dtermSetpointWeight = m.readU8();
                    // m.readU8(); // reserved
                    // m.readU8(); // reserved
                    // m.readU8(); // reserved
                    // m.readU16();
                    // m.readU16();
                    // if (m.remain() >= 2) {
                    //     _model.config.angleLimit = m.readU8();
                    //     m.readU8(); // was pidProfile.levelSensitivity
                    // }
                    // if (m.remain() >= 4) {
                    //     m.readU16(); // itermThrottleThreshold;
                    //     m.readU16(); // itermAcceleratorGain; anti_gravity_gain
                    // }
                    // if (m.remain() >= 2) {
                    //   _model.config.dtermSetpointWeight = m.readU16();
                    // }
                    // if (m.remain() >= 14) {
                    //   m.readU8(); //iterm rotation
                    //   m.readU8(); //smart feed forward
                    //   _model.config.itermRelax = m.readU8(); //iterm relax
                    //   m.readU8(); //iterm relax type
                    //   m.readU8(); //abs control gain
                    //   m.readU8(); //throttle boost
                    //   m.readU8(); //acro trainer max angle
                    //   _model.config.pid[PID_ROLL].F = m.readU16(); // pid roll f
                    //   _model.config.pid[PID_PITCH].F = m.readU16(); // pid pitch f
                    //   _model.config.pid[PID_YAW].F = m.readU16(); // pid yaw f
                    //   m.readU8(); //antigravity mode
                    // }
                    // // 1.41+
                    // if (m.remain() >= 7) {
                    //   m.readU8(); // d min roll
                    //   m.readU8(); // d min pitch
                    //   m.readU8(); // d min yaw
                    //   m.readU8(); // d min gain
                    //   m.readU8(); // d min advance
                    //   m.readU8(); // use_integrated_yaw
                    //   m.readU8(); // integrated_yaw_relax
                    // }
                    // // 1.42+
                    // if (m.remain() >= 1) {
                    //   _model.config.itermRelaxCutoff = m.readU8(); // iterm_relax_cutoff
                    // }
                    // // 1.43+
                    // if (m.remain() >= 3) {
                    //   _model.config.output.motorLimit = m.readU8(); // motor_output_limit
                    //   m.readU8(); // auto_profile_cell_count
                    //   m.readU8(); // idle_min_rpm
                    // }
                    // _model.reload();
                    break;

                case MSP_RAW_IMU:
                    //
                    //   for (int i = 0; i < 3; i++)
                    //   {
                    //     r.writeU16(lrintf(_model.state.accel[i] * ACCEL_G_INV * 512.f));
                    //   }
                    //   for (int i = 0; i < 3; i++)
                    //   {
                    //     r.writeU16(lrintf(Math::toDeg(_model.state.gyro[i])));
                    //   }
                    //   for (int i = 0; i < 3; i++)
                    //   {
                    //     r.writeU16(lrintf(_model.state.mag[i] * 1090));
                    //   }
                    //   break;
                    //
                    // case MSP_MOTOR:
                    //   for (size_t i = 0; i < OUTPUT_CHANNELS; i++)
                    //   {
                    //     if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] == -1)
                    //     {
                    //       r.writeU16(0);
                    //       continue;
                    //     }
                    //     r.writeU16(_model.state.outputUs[i]);
                    //   }
                    break;

                case MSP_SET_MOTOR:
                    // for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
                    // {
                    //   _model.state.outputDisarmed[i] = m.readU16();
                    // }
                    break;

                case MSP_SERVO:
                    //          for(size_t i = 0; i < OUTPUT_CHANNELS; i++)
                    //          {
                    //            if (i >= OUTPUT_CHANNELS || _model.config.pin[i + PIN_OUTPUT_0] ==
                    //            -1)
                    //            {
                    //              r.writeU16(1500);
                    //              continue;
                    //            }
                    //            r.writeU16(_model.state.outputUs[i]);
                    //          }
                    break;

                case MSP_SERVO_CONFIGURATIONS:
                    //          for(size_t i = 0; i < 8; i++)
                    //          {
                    //            if(i < OUTPUT_CHANNELS)
                    //            {
                    //              r.writeU16(_model.config.output.channel[i].min);
                    //              r.writeU16(_model.config.output.channel[i].max);
                    //              r.writeU16(_model.config.output.channel[i].neutral);
                    //            }
                    //            else
                    //            {
                    //              r.writeU16(1000);
                    //              r.writeU16(2000);
                    //              r.writeU16(1500);
                    //            }
                    //            r.writeU8(100);
                    //            r.writeU8(-1);
                    //            r.writeU32(0);
                    //          }
                    break;

                case MSP_SET_SERVO_CONFIGURATION:
                    //          {
                    //            uint8_t i = m.readU8();
                    //            if(i < OUTPUT_CHANNELS)
                    //            {
                    //              _model.config.output.channel[i].min = m.readU16();
                    //              _model.config.output.channel[i].max = m.readU16();
                    //              _model.config.output.channel[i].neutral = m.readU16();
                    //              m.readU8();
                    //              m.readU8();
                    //              m.readU32();
                    //            }
                    //            else
                    //            {
                    //              r.result = -1;
                    //            }
                    //          }
                    break;

                case MSP_ACC_CALIBRATION:
                    //          if(!_model.isActive(MODE_ARMED)) _model.calibrateGyro();
                    break;

                case MSP_MAG_CALIBRATION:
                    //          if(!_model.isActive(MODE_ARMED)) _model.calibrateMag();
                    break;

                case MSP_VTX_CONFIG:
                    r.writeU8(0xff);  // vtx type unknown
                    r.writeU8(0);     // band
                    r.writeU8(0);     // channel
                    r.writeU8(0);     // power
                    r.writeU8(0);     // status
                    r.writeU16(0);    // freq
                    r.writeU8(0);     // ready
                    r.writeU8(0);     // low power disarm
                    // 1.42
                    r.writeU16(0);  // pit mode freq
                    r.writeU8(0);   // vtx table available (no)
                    r.writeU8(0);   // vtx table bands
                    r.writeU8(0);   // vtx table channels
                    r.writeU8(0);   // vtx power levels
                    break;

                case MSP_SET_ARMING_DISABLED:
                    // {
                    //   const uint8_t cmd = m.readU8();
                    //   uint8_t disableRunawayTakeoff = 0;
                    //   if(m.remain()) {
                    //     disableRunawayTakeoff = m.readU8();
                    //   }
                    //   (void)disableRunawayTakeoff;
                    //   _model.setArmingDisabled(ARMING_DISABLED_MSP, cmd);
                    //   if (_model.isModeActive(MODE_ARMED))
                    //   _model.disarm(DISARM_REASON_ARMING_DISABLED);
                    // }
                    break;

                case MSP_DEBUG:
                    for (int i = 0; i < 4; i++) {
                        // r.writeU16(_model.state.debug[i]);
                    }
                    break;

                case MSP_EEPROM_WRITE:
                    // _model.save();
                    break;

                case MSP_RESET_CONF:
                    // if(!_model.isActive(MODE_ARMED))
                    // {
                    //   _model.reset();
                    // }
                    break;

                case MSP_REBOOT:
                    _reboot = true;
                    break;

                default:
                    r.result = 0;
                    break;
            }
        }

        void MspProcessor::sendResponse(MspResponse& r, Serial& s) {
            debugResponse(r, &s);
            switch (r.version) {
                case MSP_V1:
                    sendResponseV1(r, s);
                    break;
                case MSP_V2:
                    sendResponseV2(r, s);
                    break;
            }
            postCommand();
        }

        void MspProcessor::sendResponseV1(MspResponse& r, Serial& s) {
            // TODO: optimize to write at once
            uint8_t hdr[5] = {'$', 'M', '>'};
            if (r.result == -1) {
                hdr[2] = '!';  // error ??
            }
            hdr[3] = r.len;
            hdr[4] = r.cmd;
            uint8_t checksum = crc8_xor_update(0, &hdr[3], 2);
            s.write(hdr, 5);
            if (r.len > 0) {
                s.write(r.data, r.len);
                checksum = crc8_xor_update(checksum, r.data, r.len);
            }
            s.write(&checksum, sizeof(checksum));
        }

        void MspProcessor::sendResponseV2(MspResponse& r, Serial& s) {
            uint8_t hdr[8] = {'$', 'X', '>', 0};
            if (r.result == -1) {
                hdr[2] = '!';  // error ??
            }
            hdr[4] = r.cmd & 0xff;
            hdr[5] = (r.cmd >> 8) & 0xff;
            hdr[6] = r.len & 0xff;
            hdr[7] = (r.len >> 8) & 0xff;
            uint8_t checksum = crc8_dvb_s2_update(0, &hdr[3], 5);
            s.write(hdr, 8);
            if (r.len > 0) {
                s.write(r.data, r.len);
                checksum = crc8_dvb_s2_update(checksum, r.data, r.len);
            }
            s.write(&checksum, sizeof(checksum));
        }

        void MspProcessor::postCommand() {
            if (_reboot) {
                // Hardware::restart(_model);
                NVIC_SystemReset();
            }
        }

        bool MspProcessor::debugSkip(uint8_t cmd) {
            // return true;
            // return false;
            if (cmd == MSP_STATUS_EX)
                return true;
            if (cmd == MSP_BOXNAMES)
                return true;
            if (cmd == MSP_ANALOG)
                return true;
            if (cmd == MSP_ATTITUDE)
                return true;
            if (cmd == MSP_ALTITUDE)
                return true;
            if (cmd == MSP_RC)
                return true;
            if (cmd == MSP_RAW_IMU)
                return true;
            if (cmd == MSP_MOTOR)
                return true;
            if (cmd == MSP_SERVO)
                return true;
            if (cmd == MSP_BATTERY_STATE)
                return true;
            if (cmd == MSP_VOLTAGE_METERS)
                return true;
            if (cmd == MSP_CURRENT_METERS)
                return true;
            // if(cmd == USE_GPS) return true;

            return false;
        }

        void MspProcessor::debugMessage(const MspMessage& m, Serial* s) {
            if (debugSkip(m.cmd))
                return;
            // Device::SerialDevice * s = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
            if (!s)
                return;

            s->print("%c ", m.dir == MSP_TYPE_REPLY ? '>' : '<');
            s->print("%d ", m.cmd);
            s->print("%d ", m.expected);
            for (size_t i = 0; i < m.expected; i++) {
                s->print("%d ", m.buffer[i]);
            }
            s->print("\r\n");
        }

        void MspProcessor::debugResponse(const MspResponse& r, Serial* s) {
            if (debugSkip(r.cmd))
                return;
            // Device::SerialDevice* s = _model.getSerialStream(SERIAL_FUNCTION_TELEMETRY_HOTT);
            if (!s)
                return;

            s->print("%c ", r.result == 1 ? '>' : (r.result == -1 ? '!' : '@'));
            s->print("%d ", r.cmd);
            s->print("%d ", r.len);
            for (size_t i = 0; i < r.len; i++) {
                s->print("%d ", r.data[i]);
            }
            s->print("\r\n");
        }
    };  // namespace Msp
};  // namespace RMFC
