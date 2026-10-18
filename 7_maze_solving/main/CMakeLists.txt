# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "maze_solving.c" "state_follow.c" "state_junction.c" "state_turns.c"
                   "junction.c" "colour_blind.c" "drop_zone.c" "display.c"
                   "line_sensor.c" "motion.c" "pid.c" "tuning_http_server.c" "wifi_handler.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")

register_component()

spiffs_create_partition_image(www "${CMAKE_CURRENT_SOURCE_DIR}/../frontend" FLASH_IN_PROJECT)
