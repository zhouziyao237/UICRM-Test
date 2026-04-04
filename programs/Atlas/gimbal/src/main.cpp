#include "main.h"

#include "bsp_os.h"
#include "bsp_print.h"
#include "buzzer_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "minipc_task.h"
#include "rc.h"
#include "referee_task.h"
#include "robot_state.h"
#include "shoot_task.h"
#include "sys.h"
#include "ui_task.h"

void RM_RTOS_Init(void) {
    bsp::SetHighresClockTimer(&htim5);

    InitAtlasHardware();
    InitRC();
    InitIMU();
    InitBuzzer();
    InitReferee();
    // InitMiniPC();
    InitGimbal();
    InitChassis();
    InitShoot();
    if (kEnableUi) InitUI();

    print("Atlas init done. RC=%s mode=%s\r\n", RCSourceName(),
          RobotModeName(remote_mode));
}

void RM_RTOS_Threads_Init(void) {
    BuzzerTaskHandle = osThreadNew(BuzzerTask, nullptr, &BuzzerTaskAttribute);
    RCTaskHandle = osThreadNew(RCTask, nullptr, &RCTaskAttribute);
    SysTaskHandle = osThreadNew(SysTask, nullptr, &SysTaskAttribute);
    GimbalTaskHandle = osThreadNew(GimbalTask, nullptr, &GimbalTaskAttribute);
    ChassisTaskHandle = osThreadNew(ChassisTask, nullptr, &ChassisTaskAttribute);
    ShootTaskHandle = osThreadNew(ShootTask, nullptr, &ShootTaskAttribute);
    if (kEnableUi) {
        UITaskHandle = osThreadNew(UITask, nullptr, &UITaskAttribute);
    }
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    osDelay(500);

    while (true) {
        set_cursor(0, 0);
        clear_screen();

        const ImuData_t imu = ImuGetData();
        const GimbalTelemetry_t gimbal = GimbalGetTelemetry();
        const ChassisTelemetry_t chassis = ChassisGetTelemetry();
        const ShooterTelemetry_t shooter = ShooterGetTelemetry();

        print("Mode=%s killed=%d hp_dead=%d rearm=%d RC=%s\r\n",
              RobotModeName(remote_mode), RobotIsKilled() ? 1 : 0,
              robot_hp_dead ? 1 : 0, robot_rearm_required ? 1 : 0,
              RCSourceName());
        print("Gimbal init=%d online=%d output=%d pitch=%.4f yaw=%.4f\r\n",
              gimbal.initialized ? 1 : 0, gimbal.motors_online ? 1 : 0,
              gimbal.output_enabled ? 1 : 0, gimbal.pitch_relative,
              gimbal.yaw_relative);
        print("Shooter mode=%d init=%d online=%d fric=%d jam=%d\r\n",
              static_cast<int>(shooter.mode), shooter.initialized ? 1 : 0,
              shooter.online ? 1 : 0, shooter.friction_enabled ? 1 : 0,
              shooter.jammed ? 1 : 0);
        print("Chassis init=%d online=%d output=%d vx=%.2f vy=%.2f vt=%.2f\r\n",
              chassis.initialized ? 1 : 0, chassis.online ? 1 : 0,
              chassis.output_enabled ? 1 : 0, chassis.vx, chassis.vy,
              chassis.vt);
        const RemoteControl_t remote = RCGetSnapshot();
        print("Chassis input src=%s forward=%.3f right=%.3f turn=%.3f chassis_forward=%.3f chassis_right=%.3f\r\n",
              remote.keyboard_motion ? "KEYBOARD" : "STICK/IDLE",
              chassis.operator_x, chassis.operator_y, chassis.operator_turn,
              chassis.transformed_x, chassis.transformed_y);
        print("Chassis frame offset=%.1fdeg library_x(right)=%.2f library_y(forward)=%.2f\r\n",
              chassis.frame_offset_deg, chassis.library_x_speed,
              chassis.library_y_speed);
        print("Chassis yaw rel=%.3f cont=%.3f heading=%.3f hold=%d target=%.3f err=%.3f follow_err=%.3f\r\n",
              chassis.relative_yaw, chassis.relative_yaw_continuous,
              chassis.chassis_heading, chassis.heading_hold_active ? 1 : 0,
              chassis.heading_target, chassis.heading_error,
              chassis.follow_error);
        const char* power_source = chassis.power_from_referee
                                       ? "REFEREE"
                                       : (chassis.power_local_test
                                              ? "LOCAL_TEST"
                                              : "FAILSAFE");
        print("Power src=%s limit=%.1fW measured=%.2fW buffer=%.1fJ battery=%.2fV\r\n",
              power_source, chassis.power_limit_w, chassis.measured_power_w,
              chassis.power_buffer_joule, chassis.battery_voltage);

#ifdef HAS_REFEREE
        const RefereeSnapshot_t referee = RefereeGetSnapshot();
        if (referee.online) {
            print("HP=%d/%d power_limit=%dW power=%.2fW buffer=%dJ\r\n",
                  referee.hp, referee.max_hp, referee.chassis_power_limit,
                  referee.chassis_power, referee.chassis_power_buffer);
        } else {
            print("Referee: OFFLINE\r\n");
        }
#endif

        print("IMU online=%d calibrated=%d pitch=%.3f roll=%.3f yaw=%.3f temp=%.1f\r\n",
              imu.online ? 1 : 0, imu.calibrated ? 1 : 0, imu.pitch,
              imu.roll, imu.yaw, imu.temperature);
        print("Pitch logical=%.4f target=%.4f motor_theta=%.4f omega=%.4f current=%d\r\n",
              gimbal.pitch_relative, gimbal.pitch_target,
              gimbal.pitch_motor_theta, gimbal.pitch_motor_omega,
              gimbal.pitch_current);

        osDelay(100);
    }
}
