project(Atlas_gimbal ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        src/main.cpp
        src/gimbal_task.cpp
        src/minipc_task.cpp
        src/shoot_task.cpp
        src/chassis_task.cpp
        src/power_task.cpp
        src/referee_task.cpp
        src/ui_task.cpp
        src/buzzer_task.cpp
        src/imu_task.cpp
        src/sys.cpp
        src/robot_state.cpp
        src/rc.cpp

        INCLUDES include)