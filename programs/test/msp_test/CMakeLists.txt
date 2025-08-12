project(msp_test ASM C CXX)

# 收集所有源文件
file(GLOB_RECURSE SOURCES
    "*.cpp"
    "msp/*cpp"
    "Device/serial/*.cpp"
)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES ${SOURCES}
        INCLUDES . msp msp/config Model Device)
