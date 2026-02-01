project(DmMotorTest ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_main_MIT
        TARGET DJI_Board_TypeC_general
        SOURCES
        main_MIT.cpp
        INCLUDES DMmotor)

uicrm_add_arm_executable(${PROJECT_NAME}_main_MIT_Torque
        TARGET DJI_Board_TypeC_general
        SOURCES
        main_MIT_Torque.cpp
        INCLUDES DMmotor)
