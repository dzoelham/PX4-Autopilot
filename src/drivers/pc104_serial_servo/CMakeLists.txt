px4_add_module(
        MODULE drivers__pc104_serial_servo
        MAIN pc104_serial_servo
        SRCS
                pc104_serial_servo.cpp
        MODULE_CONFIG
                module.yaml
)
