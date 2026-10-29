cmake_minimum_required(VERSION 3.22)
# Ninja 同样可能只安装在 STM32Cube 的工具目录中；保留有效的显式配置。
if(CMAKE_GENERATOR STREQUAL "Ninja" AND NOT CMAKE_MAKE_PROGRAM)
    file(GLOB _stm32_ninja_dirs "$ENV{LOCALAPPDATA}/stm32cube/bundles/ninja/*")
    list(SORT _stm32_ninja_dirs COMPARE NATURAL ORDER DESCENDING)
    find_program(CMAKE_MAKE_PROGRAM NAMES ninja
        HINTS ${_stm32_ninja_dirs} PATH_SUFFIXES bin REQUIRED)
endif()
# Select the cross compiler BEFORE project(), including plain cmake -S/-B use.
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "ARM toolchain")
endif()
project(yundongv2 LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
add_executable(${PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)
option(CAMERA_UART_TEST "Build the standalone UART4 camera transmit test" OFF)
if(CAMERA_UART_TEST)
    target_sources(${PROJECT_NAME} PRIVATE Tests/camera_uart_test.c)
    target_compile_definitions(${PROJECT_NAME} PRIVATE CAMERA_UART_TEST=1)
endif()
target_sources(${PROJECT_NAME} PRIVATE
    App/Src/zdt_motor.c App/Src/zdt_motor_uart.c
    App/Src/lift_motor.c App/Src/servo_control.c
    App/Src/camera_link.c App/Src/imu_link.c
    App/Src/start_link.c App/Src/mission_runner.c App/Src/motion_control.c
    App/Src/servo_trajectory.c
    App/Src/flow_check.c
    App/Src/arm_flow.c
)
target_include_directories(${PROJECT_NAME} PRIVATE App/Inc ../../shared)
target_link_libraries(${PROJECT_NAME} stm32cubemx m)
# Camera requests contain floating-point coordinates; nano needs explicit printf support.
target_link_options(${PROJECT_NAME} PRIVATE "LINKER:-u,_printf_float")
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/STM32H743xx_FLASH.ld")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    VERBATIM)
