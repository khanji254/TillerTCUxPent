idf_component_register(SRCS "jkbms_serial_storage.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bms_monitor_types
                    REQUIRES nvs_flash
                    )

