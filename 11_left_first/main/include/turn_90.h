#ifndef TURN_90_H
#define TURN_90_H

/*
 * 90 degree turns (a junction exit or a corner, picked in state_junction.c). See turn_90.c.
 */

#include "maze.h"

void start_turn_90(maze_ctx_t *ctx, int dir);   // dir -1 left, +1 right
void handle_turn_90(maze_ctx_t *ctx);

#endif
