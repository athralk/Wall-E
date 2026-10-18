#ifndef DISPLAY_H
#define DISPLAY_H

/*
 * OLED: Kp / Ki / Kd and the IP (same as 5_line_following), plus the node
 * count "N: x" and "INV" while in the colour blind section.
 */

#include "maze.h"

/*
 * Redraws only when the PID gains changed or ctx->display_dirty is set, since
 * drawing is slow. Only called from FOLLOW and DONE so it never delays a turn.
 */
void display_update(maze_ctx_t *ctx);

#endif
