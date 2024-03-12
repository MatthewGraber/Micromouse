idf_component_register(SRCS "encoder.c" "motor_control.c" "ultrasonicLaunch.c" "robot.c" "maze.c" "main.c"
                    INCLUDE_DIRS ".")
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)