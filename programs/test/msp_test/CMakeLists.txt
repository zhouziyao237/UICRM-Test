project(msp_test ASM C CXX)

# 使用 file(GLOB) 来收集源文件
file(GLOB SOURCES "./msp/*.c" "./cli/*.c" "*.cpp")

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES ${SOURCES}
        INCLUDES .)
