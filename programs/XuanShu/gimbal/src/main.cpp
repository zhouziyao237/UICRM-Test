#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "gimbal_data.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "minipc_task.h"
#include "rc_task.h"
#include "sys_task.h"

namespace cfg = xuanshu::gimbal_config;

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);

    // Dedicated debug UART requested for the test platform.
    print_use_uart(&huart1, true, 921600);

    InitAppState();
    InitRC();
    InitIMU();
    InitMiniPC();
    InitGimbal();

    print("\r\nXuanShu dual-M6020 gimbal test app initialized\r\n");
    print("CAN1: yaw=0x%03X pitch=0x%03X\r\n",
          cfg::kYawMotorRxId, cfg::kPitchMotorRxId);
    print("RC right switch: UP=KILL MID=MANUAL DOWN=AUTOAIM\r\n");
    print("Configured encoder offsets: %s\r\n",
          cfg::kUseConfiguredEncoderOffsets ? "YES" : "NO/startup pose center");
}

void RM_RTOS_Threads_Init(void) {
    RCTaskHandle = osThreadNew(RCTask, nullptr, &RCTaskAttribute);
    SysTaskHandle = osThreadNew(SysTask, nullptr, &SysTaskAttribute);
    MiniPCTaskHandle = osThreadNew(MiniPCTask, nullptr, &MiniPCTaskAttribute);
    GimbalTaskHandle = osThreadNew(GimbalTask, nullptr, &GimbalTaskAttribute);
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(500U);

    while (true) {
        const RcSnapshot_t rc = RCGetSnapshot();
        const AppStateSnapshot_t state = AppStateGetSnapshot();
        const ImuData_t imu = ImuGetData();
        const VisionTargetSnapshot_t vision = VisionGetSnapshot();
        const GimbalTelemetry_t gimbal = GimbalGetTelemetry();

        set_cursor(0, 0);
        clear_screen();
        print("XuanShu M6020+M6020 AutoAim Test\r\n");
        print("mode=%s rearm=%d RC=%s gen=%lu\r\n",
              GimbalModeName(state.mode), state.rearm_required ? 1 : 0,
              rc.online ? "ONLINE" : "OFFLINE",
              static_cast<unsigned long>(rc.generation));
        print("manual rate pitch=% .3f yaw=% .3f rad/s\r\n",
              rc.pitch_rate_rad_s, rc.yaw_rate_rad_s);
        print("motor init=%d output=%d pitch_on=%d yaw_on=%d\r\n",
              gimbal.initialized ? 1 : 0, gimbal.output_enabled ? 1 : 0,
              gimbal.pitch_online ? 1 : 0, gimbal.yaw_online ? 1 : 0);
        print("pitch rel=% .4f theta=% .4f omega=% .3f I=%d target=% .4f\r\n",
              gimbal.pitch_relative, gimbal.pitch_motor_theta,
              gimbal.pitch_motor_omega, gimbal.pitch_current,
              gimbal.pitch_target);
        print("yaw   rel=% .4f theta=% .4f omega=% .3f I=%d target=% .4f\r\n",
              gimbal.yaw_relative, gimbal.yaw_motor_theta,
              gimbal.yaw_motor_omega, gimbal.yaw_current,
              gimbal.yaw_target);
        print("IMU online=%d cal=%d pitch=% .4f yaw=% .4f roll=% .4f temp=%.1f\r\n",
              imu.online ? 1 : 0, imu.calibrated ? 1 : 0,
              imu.pitch, imu.yaw, imu.roll, imu.temperature);
        print("Vision online=%d valid=%d acc=%u pitch=% .4f yaw=% .4f target=%u shoot=%u\r\n",
              vision.online ? 1 : 0, vision.valid ? 1 : 0,
              static_cast<unsigned int>(vision.accuracy), vision.pitch,
              vision.yaw, static_cast<unsigned int>(vision.target_robot_id),
              static_cast<unsigned int>(vision.shoot_cmd));
        print("Tune axes/offsets/PID only in gimbal_data.h\r\n");

        osDelay(cfg::kDebugPeriodMs);
    }
}
