file(GLOB_RECURSE SRCS "src/*.c" "src/*.cpp")
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES Adafruit_MLX90640
    PRIV_REQUIRES driver esp_timer
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
