#include "main.h"

#include "DMmotor.h"
#include "arm_math.h"
#include "bsp_gpio.h"
#include "bsp_print.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "fivebar.h"
#include "pid.h"

#define KEY_GPIO_GROUP KEY_GPIO_Port
#define KEY_GPIO_PIN KEY_Pin

static FiveBarLinkage* left_Linkage = nullptr;
static FiveBarLinkage* right_Linkage = nullptr;
static bsp::CAN* can2 = nullptr;
static driver::Motor6006* left_motoralpha = nullptr;
static driver::Motor6006* left_motorbeta = nullptr;

static driver::Motor6006* right_motoralpha = nullptr;
static driver::Motor6006* right_motorbeta = nullptr;

static WheelLeg::JointControl* joint_control = nullptr;
static WheelLeg::LegPos_t left_leg_pos;
static WheelLeg::LegPos_t right_leg_pos;

static WheelLeg::JointMotorData_t ConfigData[4]{
    {0, 1.50f},  // front_left
    {0, 1.50f},  // front_right
    {0, 1.50f},  // back_left
    {0, 1.50f}   // back_right
};

struct ModelLink_s {
    float L1 = 130;
    float L2 = 210;
    float L3 = 130;
    float L4 = 210;
    float L5 = 90;
} ModelLink_t;
ModelLink_s linkcfg;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    left_Linkage = new FiveBarLinkage();
    left_Linkage->setModel(linkcfg.L1, linkcfg.L2, linkcfg.L3, linkcfg.L4, linkcfg.L5);
    right_Linkage = new FiveBarLinkage();
    right_Linkage->setModel(linkcfg.L1, linkcfg.L2, linkcfg.L3, linkcfg.L4, linkcfg.L5);

    can2 = new bsp::CAN(&hcan2, false);
    right_motoralpha = new driver::Motor6006(can2, 0x05, 0x01);
    right_motoralpha->SetTransmissionRatio(1);
    right_motorbeta = new driver::Motor6006(can2, 0x06, 0x02);
    right_motorbeta->SetTransmissionRatio(1);

    left_motoralpha = new driver::Motor6006(can2, 0x07, 0x03);
    left_motoralpha->SetTransmissionRatio(1);
    left_motorbeta = new driver::Motor6006(can2, 0x08, 0x04);
    left_motorbeta->SetTransmissionRatio(1);

    control::ConstrainedPID::PID_Init_t omega_pid_init = {
        .kp = 8.0,                            // 降低Kp，避免速度环输出过大
        .ki = 0,                              // 降低Ki，减少积分饱和风险
        .kd = 2,                              // 适当微分抑制震荡
        .max_out = 1200,                      // 降低最大输出，留出安全余量 (todo: 输出范围缩放 100)
        .max_iout = 200,                      // 限制积分项
        .deadband = 0,                        // 小死区提高灵敏度
        .A = 0.5 * PI,                        // 缩小变速积分范围
        .B = 0.2 * PI,                        // 降低启动死区
        .output_filtering_coefficient = 0.1,  // 增强滤波平滑输出
        .derivative_filtering_coefficient = 0.0,
        .mode = control::ConstrainedPID::Integral_Limit |       // 积分限幅
                control::ConstrainedPID::OutputFilter |         // 输出滤波
                control::ConstrainedPID::Trapezoid_Intergral |  // 梯形积分
                control::ConstrainedPID::ChangingIntegralRate,  // 变速积分
    };

    control::ConstrainedPID::PID_Init_t theta_pid_init = {
        .kp = 12,                  // 降低Kp，避免位置环输出过大
        .ki = 0,                   // 小积分消除静差
        .kd = 4,                   // 小微分抑制震荡
        .max_out = 6 * 12.5f / 2,  // 降低位置环最大输出，为速度环留出空间
        .max_iout = 0,             // 限制积分项
        .deadband = 0,             // 提高灵敏度
        .A = 1.5 * PI,             // 关闭变速积分避免突变
        .B = 1 * PI,               // 关闭变速积分
        .output_filtering_coefficient = 0.1,
        .derivative_filtering_coefficient = 0,
        .mode = control::ConstrainedPID::OutputFilter  // 输出滤波
    };

    // 批量设置四个电机的PID参数
    driver::Motor6006* all_motors[] = {left_motoralpha, left_motorbeta, right_motoralpha,
                                       right_motorbeta};

    for (int i = 0; i < 4; i++) {
        all_motors[i]->ReInitPID(omega_pid_init, driver::DMMotorMITController::SPEED_LOOP_CONTROL);
        all_motors[i]->ReInitPID(theta_pid_init, driver::DMMotorMITController::ANGLE_LOOP_CONTROL);

        all_motors[i]->SetMode(driver::DMMotorMITController::ANGLE_LOOP_CONTROL |
                               driver::DMMotorMITController::SPEED_LOOP_CONTROL |
                               driver::DMMotorMITController::ABSOLUTE |
                               driver::DMMotorMITController::MOTOR_TORQUE_REVERSE);
    }

    WheelLeg::Joint_t joint_config;
    joint_config.JoinMotors =
        new driver::DMMotorMITController*[WheelLeg::JoinMotor::Join_motor_num];
    joint_config.JoinMotors[WheelLeg::JoinMotor::front_left] = left_motorbeta;
    joint_config.JoinMotors[WheelLeg::JoinMotor::back_left] = left_motoralpha;
    joint_config.JoinMotors[WheelLeg::JoinMotor::front_right] = right_motoralpha;
    joint_config.JoinMotors[WheelLeg::JoinMotor::back_right] = right_motorbeta;

    joint_config.Linkage = new FiveBarLinkage*[WheelLeg::TwoLink::Link_num];
    joint_config.Linkage[WheelLeg::TwoLink::left] = left_Linkage;
    joint_config.Linkage[WheelLeg::TwoLink::right] = right_Linkage;

    joint_config.data = new WheelLeg::JointMotorData_t*[WheelLeg::JoinMotor::Join_motor_num];
    for (int i = 0; i < WheelLeg::JoinMotor::Join_motor_num; i++) {
        joint_config.data[i] = &ConfigData[i];
    }
    joint_config.model = WheelLeg::CHASSIS_WHEELLEG;

    joint_config.JointMirror.SetLeft(false);
    joint_config.JointMirror.SetRight(true);

    joint_control = new WheelLeg::JointControl(joint_config);
    joint_control->Enable();

    left_leg_pos.FootEnd_X = 24.0f;
    left_leg_pos.FootEnd_Y = 50.0f;
    right_leg_pos.FootEnd_X = 24.0f;
    right_leg_pos.FootEnd_Y = 50.0f;

    HAL_Delay(1000);
    print("test demo run\n");
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);
    bsp::GPIO key(KEY_GPIO_GROUP, KEY_GPIO_PIN);
    joint_control->Enable();
    osDelay(500);
    while (true) {
        set_cursor(0, 0);
        clear_screen();

        WheelLeg::ErrorCode result = joint_control->Update();
        if (result != WheelLeg::ErrorCode::SUCCESS) {
            switch (result) {
                case WheelLeg::ErrorCode::MOTOR_OFFLINE:
                    print("[ERROR] Motor offline detected.\r\n");
                    break;
                case WheelLeg::ErrorCode::INITIALIZATION_FAILED:
                    print("[ERROR] Initialization failed.\r\n");
                    break;
                case WheelLeg::ErrorCode::UNSUPPORTED_MODE:
                    print("[ERROR] Unsupported chassis mode.\r\n");
                    break;
                default:
                    print("Unknown error occurred.\r\n");
                    break;
            }
        }

        joint_control->UpdatePosition(&left_leg_pos, &right_leg_pos);

        joint_control->GetLegPos(left_leg_pos, WheelLeg::TwoLink::left);
        joint_control->GetLegPos(left_leg_pos, WheelLeg::TwoLink::right);

        print("Left Leg Position: X=%.2f, Y=%.2f\r\n", left_leg_pos.FootEnd_X,
              left_leg_pos.FootEnd_Y);
        print("Right Leg Position: X=%.2f, Y=%.2f\r\n", right_leg_pos.FootEnd_X,
              right_leg_pos.FootEnd_Y);

        right_motorbeta->PrintData();
        right_motoralpha->PrintData();

        osDelay(100);
    }
}
