project(test_chassis ASM C CXX)

# 使用 file(GLOB) 来收集源文件
file(GLOB EEPROM_SOURCES "src/config_eeprom/*.cpp" "src/config/*.cpp")

uicrm_add_arm_executable(${PROJECT_NAME}_typeC
        TARGET DJI_Board_TypeC_general
        SOURCES
        ${EEPROM_SOURCES}
        src/main.cpp
        src/buzzer_task.cpp
        src/chassis_task.cpp
        src/system.cpp
        INCLUDES ./include ./src/.)

uicrm_add_arm_executable(${PROJECT_NAME}_typeA
        TARGET DJI_Board_TypeA_general
        SOURCES
        ${EEPROM_SOURCES}
        src/main.cpp
        src/buzzer_task.cpp
        src/chassis_task.cpp
        src/system.cpp
        INCLUDES ./include ./src/.)