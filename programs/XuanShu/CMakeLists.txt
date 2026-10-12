project(XuanShu_gimbal ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        gimbal/src/main.cpp
        gimbal/src/rc_task.cpp
        gimbal/src/sys_task.cpp
        gimbal/src/imu_task.cpp
        gimbal/src/minipc_task.cpp
        gimbal/src/gimbal_task.cpp
        INCLUDES gimbal/include)
