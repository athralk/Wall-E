#ifndef CONFIG_H
#define CONFIG_H

/*
 * Every number you might tune, in one place.
 * Duties are 0-50 (motion.c caps every wheel at 50). Times are in ms.
 */

#define LOOP_MS 10              // pause at the end of every loop

/*
 * Which part of the line to follow:
 *   0     the CENTRE of the line the whole run (normal line following)
 *   > 0   the centre for this long after the start, then the LEFT EDGE (black on the left,
 *         white from A2 on: 00111), where left branches can't be confused with wobble
 */
#define EDGE_AFTER_MS 0
#define EDGE_TARGET 2           // edge mode: A2 is the first white sensor
#define EDGE_SCALE 2.5f         // PID error per sensor the line is off (-5..+5, like before)
#define EDGE_MIN_CONTRAST 200   // brightest - darkest reading needed to trust the smooth position

/*
 * Line following speeds
 */
#define BASE_DUTY 38            // straight speed
#define MIN_DUTY 28             // slowest wheel while steering
#define MAX_DUTY 48             // fastest wheel while steering
#define KICK_DUTY 44            // start of the run: a bit more push for a moment
#define KICK_MIN_DUTY 36
#define KICK_MS 120
#define START_ON_LINE_MS 500    // motors stay off until the bar has sat on a line this long

/*
 * Corners. Centre mode: the line ran off to one side (the last outer sensor that saw it)
 * and then vanished -> turn that way. Both sides at once (T) -> left. Neither (dead end) ->
 * keep turning right round the end of the line (U-turn).
 */
#define SIDE_MEMORY_MS 400      // an outer sensor seeing the line counts for this long
#define SIDE_SAME_MS 50         // both outer sensors within this of each other = a T, go left
#define LOST_CONFIRM 2          // loops with no line before turning (rides out one bad reading)
#define CREEP_MS 80             // roll on this long first, so the wheels reach the corner
#define CREEP_DUTY 40
#define RIGHT_90_MS 650         // 90 degrees of turning, until a turn has been timed
#define RIGHT_CHECK_PCT 120     // right turn with no line by this % of 90 degrees = dead end

/*
 * Edge mode only: left branch = white under A0 and A1 (always black on the edge)
 */
#define LEFT_CONFIRM 2          // loops in a row before it counts
#define APPROACH_DUTY 40        // then drive straight so the wheels reach the junction
#define APPROACH_MS 150         // depends on the sensor bar to axle distance

/*
 * Turns in place. No stop before or after: the bot turns straight out of following and
 * goes straight back into it, easing off as the new line gets close.
 */
#define TURN_KICK_DUTY 50       // first part of every turn, gets the inner wheel reversing
#define TURN_KICK_MS 60
#define TURN_DUTY 48            // main turn speed
#define TURN_SLOW_DUTY 45       // once the new line is close. Below 45 the wheels can stall
#define TURN_STOP_EARLY 1       // hand over to the PID this many sensors before the line is
                                // centred: momentum carries it the rest (0 = right on it)
#define BRAKE_MS 0              // reverse pulse at the end of a turn (0 = none, smoothest)
#define BRAKE_DUTY 45
#define TURN_TIMEOUT_MS 2500    // nothing found: try the other way once, then stop and wait

/*
 * All white for this long: the end zone, stop
 */
#define END_WHITE_MS 1500

#endif
