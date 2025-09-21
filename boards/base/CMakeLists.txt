# generic board interface
add_library(board_interface INTERFACE
        ../../programs/test/msp_test/Module/UI/OSD.cpp
        ../../programs/test/msp_test/Module/UI/OSD.h
        ../../programs/test/msp_test/Module/UI/config/msg_data.h
        ../../programs/test/msp_test/Module/UI/Module/ChassisGUI.cpp
        ../../programs/test/msp_test/Module/UI/Module/ChassisGUI.h
        ../../programs/test/msp_test/Module/UI/Module/Bar.cpp
        ../../programs/test/msp_test/Module/UI/Module/Bar.h
        ../../programs/test/msp_test/Module/UI/Module/GimbalGUI.cpp
        ../../programs/test/msp_test/Module/UI/Module/GimbalGUI.h
        ../../programs/test/msp_test/Module/UI/Module/CapGUI.cpp
        ../../programs/test/msp_test/Module/UI/Module/CapGUI.h
        ../../programs/test/msp_test/Module/UI/Module/StringGUI.cpp
        ../../programs/test/msp_test/Module/UI/Module/StringGUI.h
        ../../programs/test/msp_test/Module/UI/Module/DiagGUI.cpp
        ../../programs/test/msp_test/Module/UI/Module/DiagGUI.h
        ../../programs/test/msp_test/Device/bsp_Serial/UARTProtocol.cpp
        ../../programs/test/msp_test/Device/bsp_Serial/UARTProtocol.h)
target_compile_definitions(board_interface INTERFACE USE_HAL_DRIVER)
target_compile_options(board_interface INTERFACE
        $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
        -ffunction-sections -fdata-sections -fmessage-length=0)
target_link_options(board_interface INTERFACE
        -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections
        -Wl,--wrap=malloc -Wl,--wrap=free -Wl,--wrap=calloc -Wl,--wrap=realloc)
target_link_libraries(board_interface INTERFACE c m)

# board specific interfaces
add_subdirectory(DJI_Board_TypeC_general)
add_subdirectory(DM_MC01_general)
add_subdirectory(DJI_Board_TypeA_general)
add_subdirectory(F103_Nano_general)

add_subdirectory(DM_MC02_general)
