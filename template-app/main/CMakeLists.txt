idf_component_register(SRCS "robot.c" "maze.c" "main.c"
                    INCLUDE_DIRS ".")
                    
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)