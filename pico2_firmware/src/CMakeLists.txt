add_executable(${PROJECT_NAME}
    main.cpp
    )

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/..
)


target_link_libraries(${PROJECT_NAME} PRIVATE
    pico_async_context_freertos
    FreeRTOS-Kernel-Heap4
    pico_stdlib
    pca9685 # ✅ Link against your library
    )


# Generate .bin, .uf2 etc.
pico_add_extra_outputs(${PROJECT_NAME})

# Enable both USB output and UART
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
