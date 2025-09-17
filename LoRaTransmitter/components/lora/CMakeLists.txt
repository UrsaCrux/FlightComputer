set(component_srcs "src/lora.c")

idf_component_register(SRCS "src/lora.c"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "include")
