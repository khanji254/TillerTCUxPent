# Add the submodule as a component
idf_component_register(
    SRCS "Src/jk_bms.c"
    INCLUDE_DIRS "Inc"
    REQUIRES modbus_rtu
)

