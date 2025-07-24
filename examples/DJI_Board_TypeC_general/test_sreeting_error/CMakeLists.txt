project(test_sreeting_error_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET ${BOARD_NAME}
        SOURCES src/m2006.cpp
                src/motor_error.cpp

        INCLUDES include)