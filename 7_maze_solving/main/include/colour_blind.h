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
 * Flips polarity when:
 *   - both outer sensors see the line and the centre doesn't (11011 / 10001)
 *     for INVERT_CONFIRM loops: works going in AND coming out, because the
 *     white line seen in inverted mode also reads as 11011
 *   - inverted and every sensor sees white (the end edge of the block) or
 *     every sensor sees black (not on the block, e.g. a mistaken flip)
 *     for INVERT_EXIT_CONFIRM loops
 * On a flip it re-reads the sensors, resets the PID and starts INVERT_HOLD_MS.
 * Returns true if it flipped this loop.
 */
bool colour_blind_update(maze_ctx_t *ctx);

#endif
