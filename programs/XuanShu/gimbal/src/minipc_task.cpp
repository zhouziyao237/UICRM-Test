#include "minipc_task.h"

#include "bsp_error_handler.h"
#include "bsp_os.h"
#include "bsp_usb.h"
#include "gimbal_data.h"
#include "imu_task.h"
#include "protocol.h"
#include "sys_task.h"
#include "cmsis_os.h"
#include "task.h"

namespace cfg = xuanshu::gimbal_config;

namespace {
bsp::VirtualUSB* usb = nullptr;
communication::Host* host = nullptr;
VisionTargetSnapshot_t frames[2]{};
volatile uint8_t active_frame = 0U;

void PublishVision(const VisionTargetSnapshot_t& target) {
    taskENTER_CRITICAL();
    const uint8_t next = static_cast<uint8_t>(active_frame ^ 1U);
    frames[next] = target;
    active_frame = next;
    taskEXIT_CRITICAL();
}
}  // namespace

osThreadId_t MiniPCTaskHandle;

void InitMiniPC() {
    usb = new bsp::VirtualUSB();
    usb->SetupRx(300);
    usb->SetupTx(300);
    communication::Protocol_t protocol = {
        .uart = nullptr,
        .usb = usb,
    };
    host = new communication::Host(protocol);
    host->SetThreshold(cfg::kMiniPCOfflineTimeoutMs);
    RM_ASSERT_TRUE(host != nullptr, "XuanShu MiniPC init failed");
}

void MiniPCTask(void* arg) {
    UNUSED(arg);
    uint32_t tx_divider = 0U;

    while (true) {
        VisionTargetSnapshot_t target;
        if (host != nullptr) {
            communication::target_angle_t raw{};
            taskENTER_CRITICAL();
            raw = host->target_angle;
            taskEXIT_CRITICAL();

            target.online = host->IsOnline();
            target.pitch = cfg::kVisionPitchSign * raw.target_pitch;
            target.yaw = cfg::kVisionYawSign * raw.target_yaw;
            target.accuracy = raw.accuracy;
            target.target_robot_id = raw.target_robot_id;
            target.shoot_cmd = raw.shoot_cmd;
            target.source_uptime_ms = host->GetLastUptime();
            target.valid = target.online &&
                           target.accuracy >= cfg::kVisionMinAccuracy;
        }
        PublishVision(target);

        // 100 Hz gimbal status upload is enough for the test platform and
        // keeps USB traffic well below the control-loop rate.
        if (++tx_divider >= 2U && host != nullptr) {
            tx_divider = 0U;
            const ImuData_t imu = ImuGetData();
            host->gimbal_current_status.current_imu_pitch = imu.pitch;
            host->gimbal_current_status.current_imu_roll = imu.roll;
            host->gimbal_current_status.current_imu_yaw = imu.yaw;
            host->gimbal_current_status.robot_id = 0U;
            host->gimbal_current_status.shooter_id = 0U;
            // One writer, one packet at a time. The test app deliberately
            // avoids back-to-back USB Transmit() calls.
            host->Transmit(communication::GIMBAL_CURRENT_STATUS);
        }

        osDelay(cfg::kMiniPCPeriodMs);
    }
}

VisionTargetSnapshot_t VisionGetSnapshot() {
    taskENTER_CRITICAL();
    const VisionTargetSnapshot_t target = frames[active_frame];
    taskEXIT_CRITICAL();
    return target;
}
