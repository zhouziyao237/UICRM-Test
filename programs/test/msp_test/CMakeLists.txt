project(msp_test ASM C CXX)

# 收集所有源文件
file(GLOB_RECURSE SOURCES
        "./*.cpp"
        "Common/*.cpp"
        "Module/*.cpp"
        "Module/RFS/*.cpp"
        "Module/MiniPC/*.cpp"
        "Module/RX/*.cpp"
        "Module/RX/Module/*.cpp"
        "Module/RX/Module/sbus/*.cpp"
        "Module/Msp/*.cpp"
        "Module/Cli/*.cpp"
        "Module/UI/*.cpp"
        "Module/UI/Module/*.cpp"
        "Device/*.cpp"
        "System/*.cpp"
        "PG/*.cpp"
        "Task/RXTask/*cpp"
        "Task/MSPTask/*cpp"
        "Task/RFSTask/*cpp"
        "Task/MiniPCTask/*cpp"
)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES ${SOURCES}
        INCLUDES . Common PG Module Module/Msp Module/MiniPC Module/RX Module/Cli Module/UI Module/RFS Device System)
