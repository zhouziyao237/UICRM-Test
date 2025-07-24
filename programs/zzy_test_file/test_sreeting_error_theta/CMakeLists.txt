project(test_sreeting_error_theta_TypeA ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES m2006_theta.cpp)