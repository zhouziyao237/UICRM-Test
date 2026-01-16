project(DmMotorTest ASM C CXX)

# 收集所有源文件
file(GLOB_RECURSE SOURCES
        "./DMmotor/*.cpp"
        "./*.cpp"
)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeC_general
        SOURCES ${SOURCES}
        INCLUDES . DMmotor)
