#ifndef JUNCTION_H
#define JUNCTION_H

/*
 * Pattern checks and the exit choice used by FOLLOW and APPROACH.
 * Nothing here moves the motors or changes state.
 */

#include <stdbool.h>
#include <stdint.h>
#include "config.h"
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
 * Normal width of the line (sensors lit), learned from plain readings: one block of lit
 * sensors that doesn't touch an outer sensor, so no branch can be in it. The typical width
 * is the widest of the last LINE_WIDTH_HISTORY plain readings (the same line lights 2 or 3
 * sensors depending on where it sits, the widest is what it can really cover).
 */
typedef struct
{
    uint8_t width[LINE_WIDTH_HISTORY];
    int count;      // readings stored so far (up to LINE_WIDTH_HISTORY)
    int next;       // where the next one goes
} line_width_t;

void line_width_reset(line_width_t *lw);
void line_width_update(line_width_t *lw, const lsa_state_t *lsa);
int line_width_typical(const line_width_t *lw);

/*
 * Side branches under the bar, LSA only (the side IR is handled by the callers).
 * A branch on a side = ONE block of lit sensors that
 *   - reaches that side's outer sensor,
 *   - still covers the centre sensor (the main line is there), and
 *   - is wider than the line can ever be on its own (typical_width + 1 or more).
 * A line that only slid sideways keeps its width (01110 -> 11100 is 3 wide), so it can
 * never count, however far it slides. 11110 / 11111 from a 3 wide line, or 11100 from a
 * 1 wide line, are real branches. Two separate blocks are never a branch (see detect_side_line).
 * was_centred: the previous reading had the line in the middle, then the normal width is
 * enough (a corner coming under the bar, not a slide).
 */
int detect_branches(const lsa_state_t *lsa, int typical_width, bool was_centred);

/*
 * A second, separate line beside the main one (a gap between two lit blocks, e.g. 10110):
 * the arm of an acute (~120 degree) exit sweeping under the bar. Returns BRANCH_LEFT /
 * BRANCH_RIGHT for the side the separate block touches. One line on its own never has
 * a gap, so wobble can't cause this.
 */
int detect_side_line(const lsa_state_t *lsa);

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
 * Picks an exit at a node using EXIT_RULE (config.h). For RULE_STRAIGHT_FIRST:
 * straight first, then TURN_PREFERENCE, then the other turn, and after a U-turn
 * straight is tried last.
 * plan ('S' / 'L' / 'R') forces an exit if that exit exists.
 */
exit_t choose_exit(bool left, bool straight, bool right, bool after_uturn, char plan);

const char *exit_name(exit_t exit);

#endif
