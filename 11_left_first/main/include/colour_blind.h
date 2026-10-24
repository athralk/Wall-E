#ifndef COLOUR_BLIND_H
#define COLOUR_BLIND_H

/*
 * Colour blind section: a white block with a black line down the middle.
 *
 * Instead of a separate state, the sensor polarity is flipped so black counts
 * as the line, and FOLLOW keeps working as normal (PID steers toward the black
 * line, not away from it).
 */

#include <stdbool.h>
#include "maze.h"

/*
 * Call every loop while in FOLLOW / APPROACH, after line_sensor_update().
 * Raw patterns, 1 = white:
 *   - white line mode, 10001 (or 11011) for INVERT_CONFIRM loops -> black line following
 *   - black line mode, 01110 (or 00100) for INVERT_EXIT_CONFIRM loops -> white line following
 *     (also if the bar sees no white at all: off the block, e.g. a mistaken flip)
 * On a flip it re-reads the sensors, resets the PID and starts INVERT_HOLD_MS.
 * Returns true if it flipped this loop.
 */
bool colour_blind_update(maze_ctx_t *ctx);

#endif
