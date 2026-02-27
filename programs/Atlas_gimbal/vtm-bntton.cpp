#include "RCKEY.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "vtm.h"

// ==================== RC系统主体 ====================

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 26

// RC数据数组
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {0};

// VTM实例
static bsp::UART* uart = nullptr;
static remote::VTM* vtm;

// RC任务
osThreadId_t rc_task_handle;

const osThreadAttr_t RCTaskAttribute = {
    .name = "RCTask",
    .attr_bits = osThreadDetached,
    .cb_mem = nullptr,
    .cb_size = 0,
    .stack_mem = nullptr,
    .stack_size = 512 * 2,
    .priority = (osPriority_t)osPriorityHigh,
    .tz_module = 0,
    .reserved = 0
};

// RC更新函数
void rc_update_task(void* argument) {
    UNUSED(argument);
    while (1) {
        if (!vtm) {
            // 初始化失败处理
            osDelay(100);
            continue;
        }

        if (!vtm->IsOnline()) {
            // 连接断开处理
            osDelay(100);
            continue;
        }

        // 更新RC数据
        rcData[0] = static_cast<float>(vtm->ch0);   // ROLL
        rcData[1] = static_cast<float>(vtm->ch1);   // PITCH
        rcData[2] = static_cast<float>(vtm->ch2);   // YAW
        rcData[3] = static_cast<float>(vtm->ch3);   // THROTTLE
        rcData[4] = static_cast<float>(vtm->ch4);   // AUX1
        rcData[5] = static_cast<float>(vtm->ch5);   // AUX2

        // 使用RC按键库处理按键通道
        rc_key_process(4, (uint16_t)(vtm->ch5 ? 2000 : 1000));
        rc_key_process(5, (uint16_t)(vtm->custom_left ? 2000 : 1000));
        rc_key_process(6, (uint16_t)(vtm->custom_right ? 2000 : 1000));

        // 处理待触发事件
        rc_key_process_events();

        osDelay(1);
    }
}

// 初始化函数
void RM_RTOS_Init(void) {
    // 初始化VTM
    print_use_uart(&huart1, true, 921600);
    uart = new bsp::UART(&huart6);
    uart->SetBaudrate(912600);
    uart->SetupRx(BUFFER_SIZE);
    vtm = new remote::VTM(uart);

    // 初始化RC按键库
    rc_key_init();

    // 配置按键
    rc_key_config_t configs[] = {
        {4, 1750, 1250, 1000, 200},  // 暂停按钮
        {5, 1750, 1250, 1500, 200},  // 左键
        {6, 1750, 1250, 800, 200}    // 右键
    };

    for (int i = 0; i < 3; i++) {
        rc_key_add(configs[i].channel, configs[i]);
    }

    // 注册回调
    rc_key_register_callback([](uint8_t channel, rc_key_mode_t mode) {
        const char* modes[] = {"无", "单击", "双击", "长按", "释放"};
        print("通道%d: %s\r\n", channel, modes[mode]);
    });

}

void RM_RTOS_Threads_Init(void) {
    // 分别启动每个任务
    rc_task_handle = osThreadNew(rc_update_task, nullptr, &RCTaskAttribute);
}

// ==================== 使用示例 ====================

void RM_RTOS_Default_Task(const void* arg){
    UNUSED(arg);
    print("=== RC系统测试 ===\r\n");

    print("系统初始化完成\r\n");
    print("支持功能：\r\n");
    print("- 通道4: 暂停按钮\r\n");
    print("- 通道5: 自定义左键\r\n");
    print("- 通道6: 自定义右键\r\n");

    // 主循环
    while(1) {
        set_cursor(0, 0);
        clear_screen();
        if (vtm && vtm->IsOnline()) {
            print("RC连接正常\r\n");
        } else {
            print("RC连接断开\r\n");
        }

        char buf[512];
        vTaskList(buf);
        print("\r\nvTaskList: \r\n");
        print("%s\r\n", buf);

        osDelay(500);
    }
}
