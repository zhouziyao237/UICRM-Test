#include "Kinematics.h"
#include "bsp_print.h"
#include "main.h"

static Kinematics kinematics;

void RM_RTOS_Init(void) {
    print_use_uart(&huart1, true, 921600);

    // 设置机构参数
    Kinematics::Model model;
    model.L1 = 45.0f;
    model.L2 = 80.0f;
    model.L3 = 45.0f;
    model.L4 = 80.0f;
    model.L5 = 55.0f;
    kinematics.setModel(model);

}
void RM_RTOS_Threads_Init(void) {
}

void RM_RTOS_Default_Task(const void* arg) {
    UNUSED(arg);


    while (true) {
        // 逆运动学计算（根据足端坐标计算关节角度）
        float x = 100.0f;  // 足端X坐标
        float y = 50.0f;   // 足端Y坐标
        Kinematics::IKResult ikResult = kinematics.inverseKinematics(x, y);
        print("逆解结果：\n");
        print("alpha: %.4f rad (%.1f°)\n", ikResult.alpha, ikResult.alpha * 180.0f / PI);
        print("beta: %.4f rad (%.1f°)\n", ikResult.beta, ikResult.beta * 180.0f / PI);
        print("中点到足端距离: %.4f mm\n", ikResult.midpointToEndDistance);
        print("中点到足端角度: %.4f rad (%.1f°)\n", ikResult.midpointToEndAngle,
              ikResult.midpointToEndAngle * 180.0f / PI);

        // 正运动学计算（根据关节角度计算足端坐标）
        float alpha = ikResult.alpha;  // 左侧关节角度
        float beta = ikResult.beta;    // 右侧关节角度
        Kinematics::FKResult fkResult = kinematics.forwardKinematics(alpha, beta);
        if (fkResult.hasSolution) {
            print("正解结果：\n");
            print("交点1: (%.4f, %.4f)\n", fkResult.x1, fkResult.y1);
            print("交点2: (%.4f, %.4f)\n", fkResult.x2, fkResult.y2);
            print("中点到足端距离: %.4f mm\n", fkResult.midpointToEndDistance);
            print("中点到足端角度: %.4f rad (%.1f°)\n", fkResult.midpointToEndAngle,
                  fkResult.midpointToEndAngle * 180.0f / PI);
        } else {
            print("正解无可行解\n");
        }
    }
}