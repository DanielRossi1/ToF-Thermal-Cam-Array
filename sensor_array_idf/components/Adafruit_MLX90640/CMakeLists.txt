idf_component_register(
    SRCS
        "MLX90640_API.c"
        "mlx90640_idf_hal.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver freertos
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-variable
    -Wno-sign-compare
    -Wno-implicit-fallthrough
)
