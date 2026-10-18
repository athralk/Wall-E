#ifndef JUNCTION_H
#define JUNCTION_H

/*
 * Pattern checks and the exit choice used by FOLLOW and APPROACH.
 * Nothing here moves the motors or changes state.
 */

#include <stdbool.h>
#include "line_sensor.h"

/*
 * Branch flags returned by detect_branches()
 */
#define BRANCH_LEFT 0x1
#define BRANCH_RIGHT 0x2

typedef enum
{
    EXIT_NONE,
    EXIT_LEFT,
    EXIT_STRAIGHT,
    EXIT_RIGHT,
} exit_t;

/*
 * Side branches under the bar: 11100 = left, 00111 = right, 11111 = both
 * (T, cross or the end zone)
 */
int detect_branches(const lsa_state_t *lsa);

/*
 * 45 degree turn: line only under the outermost sensor(s) of one side.
 * Returns -1 for left, +1 for right, 0 otherwise
 */
int detect_sharp(const lsa_state_t *lsa);

/*
 * Counts how many loops in a row the same non-zero direction was seen, so one
 * noisy reading can't trigger a turn. Returns the count.
 */
int confirm(int dir, int *last_dir, int *count);

/*
 * NODE_PLAN entry for this node (1 = first node), 'A' if the plan is shorter
 */
char node_plan_for(int node);

/*
 * Picks an exit at a node. Straight first, then TURN_PREFERENCE, then the other turn.
 * After a U-turn straight leads back to where we came from, so it is tried last.
 * plan ('S' / 'L' / 'R') forces an exit if that exit exists.
 */
exit_t choose_exit(bool left, bool straight, bool right, bool after_uturn, char plan);

const char *exit_name(exit_t exit);

#endif
