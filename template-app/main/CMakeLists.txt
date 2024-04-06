idf_component_register(SRCS "LED.c" "icm20948.c" "ultrasonicLaunch.c" "encoder.c" "motor_control.c" "robot.c" "maze.c" "main.c"
                    INCLUDE_DIRS ".")
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)