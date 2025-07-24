project(test_sreeting_error_TypeA ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES src/m2006.cpp
                src/motor_error.cpp

        INCLUDES include)