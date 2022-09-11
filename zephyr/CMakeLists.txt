if (CONFIG_DRIVER_WS2812B)
add_library(DRIVER_WS2812B INTERFACE)

zephyr_include_directories(../Drivers/WS2812B/Inc)

zephyr_library()

zephyr_library_sources(../Drivers/WS2812B/Src/ws2812b.c)

endif()