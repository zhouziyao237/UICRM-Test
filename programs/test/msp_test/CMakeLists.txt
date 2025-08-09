project(msp_test ASM C CXX)

# 收集所有源文件
file(GLOB_RECURSE SOURCES
    "*.cpp"
    "cli/*.cpp"
    "msp/*cpp"
    "serial/*.cpp"
    "Model/*.cpp"
)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES ${SOURCES}
        INCLUDES . cli msp msp/config Model serial)
