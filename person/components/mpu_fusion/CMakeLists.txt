idf_component_register(SRCS "src/quaternions.c" 
                       INCLUDE_DIRS "include" 
                       REQUIRES mbedtls 
                       PRIV_REQUIRES driver esp_timer)
