idf_component_register(
    SRCS
        "vl53l8cx_api.c"
        "vl53l8cx_platform.c"
        "vl53l8cx_plugin_detection_thresholds.c"
        "vl53l8cx_plugin_motion_indicator.c"
        "vl53l8cx_plugin_xtalk.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver esp_timer freertos
)

# The ULD references standard C libs — make them available.
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-variable
    -Wno-unused-but-set-variable
    -Wno-implicit-fallthrough
)
