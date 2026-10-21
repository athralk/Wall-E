#ifndef CONFIG_H
#define CONFIG_H

/*
 * Every number you might tune, in one place.
 * Duties are 0-50 (motion.c caps every wheel at 50). Times are in ms.
 */

#define LOOP_MS 10              // pause at the end of every loop

/*
 * Edge following. The bot rides the LEFT edge of the white line:
 * black under A0 A1, white from A2 on (00111). EDGE_TARGET is that first white sensor.
 */
#define EDGE_TARGET 2
#define EDGE_SCALE 2.5f         // PID error per sensor the edge is off (same -5..+5 range as before)
#define EDGE_MIN_CONTRAST 200   // brightest - darkest reading needed to trust the smooth edge position
#define BASE_DUTY 30            // straight speed
#define MIN_DUTY 20             // slowest wheel while steering
#define MAX_DUTY 38             // fastest wheel while steering

/*
 * Pulling away after a stop (start, after every turn): a bit more push for a moment,
 * so the wheels don't sit still at BASE_DUTY
 */
#define KICK_DUTY 38
#define KICK_MIN_DUTY 30
#define KICK_MS 120
#define START_ON_LINE_MS 500    // motors stay off until the bar has sat on a line this long

/*
 * Left branch: white under A0 and A1, which are always on black while following
 */
#define LEFT_CONFIRM 2          // loops in a row before it counts
#define APPROACH_DUTY 38        // then drive straight so the wheels reach the junction
#define APPROACH_MS 150         // depends on the sensor bar to axle distance
                                // (only once the bot has found the edge again after a turn)

/*
 * Line lost (00111 -> 00000: right corner, dead end, or drifted left): pivot right in place.
 * Edge back within ~90 degrees = right turn. Not by then = dead end: keep turning right,
 * round the end of the line and onto its other side (the U-turn).
 */
#define LOST_CONFIRM 2          // loops of 00000 before turning (rides out one bad reading)
#define RIGHT_90_MS 650         // 90 degrees of turning, until a left turn has been timed
#define RIGHT_CHECK_PCT 120     // look for the right turn this % of a 90 degree turn

/*
 * Turns in place
 */
#define TURN_KICK_DUTY 50       // start of every turn, to get the wheels going
#define TURN_KICK_MS 60
#define TURN_DUTY 45            // main turn speed. Below 45 the wheels can stall
#define TURN_SLOW_DUTY 40       // once the new line is close, ease onto it
#define TURN_STOP_EARLY 1       // stop this many sensors before the edge: momentum and the
                                // brake carry it the rest (0 = stop exactly on the edge)
#define TURN_TIMEOUT_MS 3000    // found nothing in this long: stop and wait to be put back
#define BRAKE_DUTY 45           // short reverse pulse at the end of a turn, kills overshoot
#define BRAKE_MS 30

/*
 * All white (wide junction, landed on the line after a turn): the PID steers left onto the
 * edge by itself. Still all white after this long: the end zone, stop.
 */
#define END_WHITE_MS 1500

#endif
