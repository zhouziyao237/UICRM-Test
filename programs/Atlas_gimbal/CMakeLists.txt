project(Atlas_gimbal)

uicrm_add_arm_executable(${PROJECT_NAME}demo2
        TARGET  DJI_Board_TypeC_general
        SOURCES typeC-demo2.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}m3508_example
        TARGET  DJI_Board_TypeC_general
        SOURCES m3508_theta.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_m2006_example
        TARGET DJI_Board_TypeC_general
        SOURCES m2006_theta.cpp)

uicrm_add_arm_executable(${PROJECT_NAME}_vtm-bntton
        TARGET DJI_Board_TypeC_general
        SOURCES
        vtm-bntton.cpp)

uicrm_add_arm_executable(Atlas_Gimbal
        TARGET  DJI_Board_TypeC_general
        SOURCES
        control/main.cpp
        comm/rotation_utils.cpp
        sensors/rx/rx_vtm.cpp
        sensors/gyro/gyrobmi088.cpp
        tesk/Gyro_task.cpp
        tesk/remote_task.cpp

        INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/comm
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/gyro/
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors/rx
        ${CMAKE_CURRENT_SOURCE_DIR}/sensors
        ${CMAKE_CURRENT_SOURCE_DIR}/tesk
        ${CMAKE_CURRENT_SOURCE_DIR}/taget/Atlas
        ${CMAKE_CURRENT_SOURCE_DIR}/taget)
