px4_add_module(
        MODULE drivers__pc104_sensorhub
        MAIN pc104_sensorhub
        SRCS
                pc104_sensorhub.cpp
        MODULE_CONFIG
                module.yaml
)
