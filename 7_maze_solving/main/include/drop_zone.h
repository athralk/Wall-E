#ifndef DROP_ZONE_H
#define DROP_ZONE_H

/*
 * Pink drop zone (object push task).
 *
 * The bot just follows the line and pushes the object ahead of it. When the
 * middle sensors read pink (PINK_MIN..PINK_MAX in config.h) the object is in
 * the zone, so DROP stops, backs off a little and hands over to UTURN to go
 * back into the maze. Only happens once per run.
 */

#include "maze.h"

typedef enum
{
    DROP_NONE,          // not pink
    DROP_MAYBE,         // pink this loop, not confirmed yet
    DROP_CONFIRMED,     // pink for PINK_CONFIRM loops
} drop_check_t;

/*
 * Call every loop from FOLLOW / APPROACH. Always DROP_NONE once the drop is done.
 */
drop_check_t drop_zone_check(maze_ctx_t *ctx);

/*
 * Enters DROP and stops the motors
 */
void start_drop(maze_ctx_t *ctx);

#endif
