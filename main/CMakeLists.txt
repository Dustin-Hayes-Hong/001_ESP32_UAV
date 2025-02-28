idf_component_register( SRCS            "main.c"
                        INCLUDE_DIRS    "."
                        REQUIRES        "inside_communication"
                                        "sensor"
                                        "LED"
                                        "VBAT"
                                        "external_communication"
                                        "flight_controller"
                        )