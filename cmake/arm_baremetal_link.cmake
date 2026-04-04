# Keep GNU Arm Embedded builds independent from the Windows PE platform rules.
# A stale CMake cache on Windows can otherwise append --out-implib and
# --major-image-version to arm-none-eabi-ld, which only supports bare-metal ELF.

get_filename_component(_arm_c_compiler_name "${CMAKE_C_COMPILER}" NAME)
get_filename_component(_arm_cxx_compiler_name "${CMAKE_CXX_COMPILER}" NAME)

if(_arm_c_compiler_name MATCHES "^arm-none-eabi-gcc(\\.exe)?$" AND
   _arm_cxx_compiler_name MATCHES "^arm-none-eabi-g\\+\\+(\\.exe)?$")
    set(CMAKE_EXECUTABLE_SUFFIX "")

    set(CMAKE_C_LINK_EXECUTABLE
        "<CMAKE_C_COMPILER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
    set(CMAKE_CXX_LINK_EXECUTABLE
        "<CMAKE_CXX_COMPILER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>")
else()
    message(FATAL_ERROR
        "UICRM STM32 build requires arm-none-eabi-gcc/g++. "
        "C compiler: ${CMAKE_C_COMPILER}; CXX compiler: ${CMAKE_CXX_COMPILER}")
endif()

unset(_arm_c_compiler_name)
unset(_arm_cxx_compiler_name)
