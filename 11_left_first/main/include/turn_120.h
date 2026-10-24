#ifndef TURN_120_H
#define TURN_120_H

/*
 * Acute (~120 degree) turns: the exit runs back the way the bot came. See turn_120.c.
 */

#include "maze.h"

void start_turn_120(maze_ctx_t *ctx, int dir);  // dir -1 left, +1 right
void handle_turn_120(maze_ctx_t *ctx);

#endif
