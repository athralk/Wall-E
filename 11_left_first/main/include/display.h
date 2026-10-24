#ifndef DISPLAY_H
#define DISPLAY_H

/*
 * OLED: Kp / Ki / Kd and the IP (same as 5_line_following), plus the node
 * count "N: x" and "INV" while in the colour blind section.
 */

#include "maze.h"

/*
 * Redraws only when the PID gains changed or ctx->display_dirty is set, since
 * drawing is slow (tens of ms over I2C). Called from its own task (maze_solving.c),
 * never from the control loop, so a redraw can't delay reading the LSA or driving.
 */
void display_update(maze_ctx_t *ctx);

/*
 * Calibration mode (CALIBRATE_LSA): the 5 readings and the pattern as text
 */
void display_readings(const lsa_state_t *lsa);

#endif
