idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver
    PRIV_REQUIRES nvs_flash
)
