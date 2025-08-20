project(msp_test ASM C CXX)

# 收集所有源文件
file(GLOB_RECURSE SOURCES
        "./*.cpp"
        "Module/*.cpp"
        "Module/RFS/*.cpp"
        "Common/*.cpp"
        "Msp/*.cpp"
        "Device/*.cpp"
        "System/*.cpp"
        "PG/*.cpp"
        "Cli/*.cpp"
)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES ${SOURCES}
        INCLUDES . Common Msp PG Cli Module Module/RFS Device System)
