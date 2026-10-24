#ifndef TURN_90_H
#define TURN_90_H

/*
 * 90 degree turns (a junction exit or a corner, picked in state_junction.c) and the
 * U-turn at a dead end (two 90s in one go). Both pivot in place. See turn_90.c.
 */

#include "maze.h"

void start_turn_90(maze_ctx_t *ctx, int dir);   // dir -1 left, +1 right
void start_uturn(maze_ctx_t *ctx);              // spins UTURN_DIR
void handle_turn_90(maze_ctx_t *ctx);

#endif
