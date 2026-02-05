
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")

uicrm_add_arm_executable(Wheel-Leg
        TARGET DJI_Board_TypeC_general
        SOURCES
        main.cpp
        ./Kinematics/kinematics.cpp
        ./Kinematics/Model/fivebar.cpp
        ./Chassis/chassis.cpp

        INCLUDES ./Chassis ./Kinematics ./Kinematics/Model
)
