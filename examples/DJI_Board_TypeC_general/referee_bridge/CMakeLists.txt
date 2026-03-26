project(example_referee_bridge_${BOARD_NAME} ASM C CXX)

uicrm_add_arm_executable(${PROJECT_NAME}_host
        TARGET ${BOARD_NAME}
        SOURCES host_main.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_slave
        TARGET ${BOARD_NAME}
        SOURCES slave_main.cpp)