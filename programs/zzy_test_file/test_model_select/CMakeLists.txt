project(test_model_select)

uicrm_add_arm_executable(${PROJECT_NAME}
        TARGET DJI_Board_TypeA_general
        SOURCES src/main.cpp
                src/model.cpp
        INCLUDES include)