#ifndef CONFIG_H
#define CONFIG_H

/*
 * Every tunable number for the maze in one place. This is the only file you
 * should need to edit while tuning on the track.
 *
 * Duties are motor duty cycles (motion.c caps each wheel at 50).
 * Times are in milliseconds, "loops" are runs of the control loop (LOOP_MS each).
 */

#define LOOP_MS 10              // control loop period
#define PID_REAL_DT 0           // 1 = PID derivative uses the measured loop time (micromouse style), 0 = per loop as before

/*
 * Line following speeds (FOLLOW state)
 */
#define BASE_DUTY 30
#define MIN_DUTY 20
#define MAX_DUTY 45

/*
 * Line centering (like wall centering on the micromouse): 1 = PID error from the analog
 * readings, smooth. 0 = old on / off sensor position, only a few steps.
 */
#define ANALOG_CENTERING 0      // off for now: the known-good on / off position, try 1 once turns are reliable
#define MIN_CONTRAST 150        // brightest - darkest sensor must differ this much to trust the analog position

/*
 * 90 degree corners: line suddenly covers one side of the bar (11100 / 00111).
 * Drive forward until the axle is over the corner, then pivot in place onto the new line.
 */
#define CORNER_CONFIRM 2        // loops in a row a side branch pattern must be seen before we treat it as a corner / node
#define APPROACH_DUTY 40        // straight drive to bring the axle over the corner. Too low and the bot stalls on the junction
#define APPROACH_MS 150         // depends on sensor bar to axle distance, tune on track
#define TURN_DUTY 45            // pivot speed (50 is the max motion.c allows). Below 45 the motors stall in a pivot
#define PIVOT_MIN_MS 40         // ignore the line for this long after starting a pivot
#define PIVOT_TIMEOUT_MS 1500   // give up and search if the new line never shows
#define BRAKE_DUTY 40           // short reverse pulse to kill pivot overshoot
#define BRAKE_MS 40
#define REARM_LOOPS 5           // plain line readings needed after a turn before the next turn can trigger
#define END_CONFIRM_MS 200      // end zone: bar must stay all white this much longer (still driving) before stopping

/*
 * Acute turns: the line slides out to the outermost sensor faster than PID can follow
 * (10000 / 00001). Treated as a ~120 degree turn: pivot toward it, ignore the line for
 * SHARP_MIN_PCT of a 90 degree pivot (so the bot can't stop on the line it came in on or
 * half way), then stop as soon as the centre finds the line.
 */
#define SHARP_CONFIRM 2
#define SHARP_DUTY 45           // pivot speed. Below 45 the motors stall in a pivot
#define SHARP_MIN_PCT 100       // ignore the line for this % of a 90 degree pivot: 100 = first ~90 degrees.
                                // Lower it (e.g. 50) if gentle 45 degree bends now spin past their line
#define SHARP_TIMEOUT_MS 1500

/*
 * Which exit to take at a node (junction with 2+ exits):
 *   RULE_RIGHT_HAND      right, then straight, then left (right wall follower). Never needs
 *                        to come back to a node: dead ends sort themselves out
 *   RULE_LEFT_HAND       left, then straight, then right (left wall follower)
 *   RULE_STRAIGHT_FIRST  straight, then TURN_PREFERENCE, then the other turn. Every node is
 *                        remembered (node_memory.c): if the exit taken is a dead end, the bot
 *                        comes back to the node and takes the next exit it hasn't tried
 */
#define RULE_STRAIGHT_FIRST 0
#define RULE_RIGHT_HAND 1
#define RULE_LEFT_HAND 2
#define EXIT_RULE RULE_STRAIGHT_FIRST

#define TURN_PREFERENCE -1      // RULE_STRAIGHT_FIRST only: turn to take when there is no straight, -1 left, +1 right
/*
 * Optional per-node override, one character per node in the order they are counted:
 * 'A' = auto (rules above), 'S' = straight, 'L' = left, 'R' = right.
 * e.g. "AARA" forces a right turn at node 3. Nodes past the end of the string are auto.
 * If the forced exit doesn't exist at that node, auto is used.
 */
#define NODE_PLAN ""

/*
 * Dead end: line disappears while it was centred under the bar. Spin 180 degrees.
 * Also used after the drop zone.
 */
#if EXIT_RULE == RULE_RIGHT_HAND
#define UTURN_DIR 1             // spin right, keeping the "hand" on the right wall
#elif EXIT_RULE == RULE_LEFT_HAND
#define UTURN_DIR -1
#else
#define UTURN_DIR TURN_PREFERENCE
#endif
#define UTURN_MIN_MS 400        // ignore the line for this long so we don't grab the stub we just left
#define UTURN_TIMEOUT_MS 3000

/*
 * Dead end swing: before U-turning, swing SWING_PCT of 90 degrees to TURN_PREFERENCE's side,
 * then the same past forward to the other side, looking for an acute (~120-135 degree) exit
 * the bar ran past. Nothing found: carry on round to the line we came in on (a U-turn).
 * Angles are timed from how long the last 90 degree pivot took (PIVOT_90_MS until the
 * first pivot has been timed).
 */
#define DEADEND_SWING 1         // 0 = plain U-turn at dead ends, like before
#define SWING_PCT 150           // swing angle in % of 90 degrees: 150 = 135 degrees each side
#define PIVOT_90_MS 450         // guess for a 90 degree pivot at TURN_DUTY, used before one is timed

/*
 * Lost line recovery (gaps, acute turns sharper than 90)
 */
#define LOST_CONFIRM 3          // loops with no line before we start searching
#define DEADEND_CONFIRM 5       // loops with no line (driving straight) before a dead end U-turn
#define START_HOLD_MS 1000      // at power on: drive straight until the line is found (ends as soon as it is)
#define SEARCH_DUTY 45          // pivots need more power than driving: below ~40 the bot may not move
#define SEARCH_TIMEOUT_MS 800   // flip search direction after this, doubles each flip
#define SEARCH_UTURN_MS 700     // a search that spun this long one way found the line behind: counts as
                                // a dead end U-turn. Only used until the first 90 degree pivot is timed,
                                // then 1.5x that pivot time is used instead

/*
 * End zone stop. If the bot stopped by mistake, put it back on the line: after the line has
 * been under the bar for RESUME_MS it carries on by itself (also handy for checkpoint repositions).
 */
#define RESUME_MS 1000

/*
 * Colour blind section: black line on a white block. When both outer sensors see the line
 * and the centre doesn't (11011 / 10001), the polarity is wrong, so it gets flipped.
 */
#define INVERT_CONFIRM 3        // loops the 1x0x1 pattern must hold before flipping
#define INVERT_EXIT_CONFIRM 2   // loops of all white or all black (while inverted) before flipping back
#define INVERT_HOLD_MS 300      // after a flip, only PID straight: no junction / sharp / dead end checks

/*
 * Pink drop zone. Pink reads somewhere between black and white on the LSA.
 * Calibrate: set LOG_READINGS to 1, hold the bar over the pink square, and set the band
 * around the values printed for the middle three sensors.
 */
#define DROP_ZONE_ENABLED 0     // 0 until PINK_MIN / PINK_MAX are calibrated, guessed values can fire on black
#define PINK_MIN 600            // normalised reading (0 black .. 1000 white), PLACEHOLDER
#define PINK_MAX 800            // PLACEHOLDER
#define PINK_CONFIRM 5          // loops the middle three sensors must stay in the band
#define DROP_STOP_MS 500        // stand still so the object settles in the zone
#define DROP_REVERSE_DUTY 30
#define DROP_REVERSE_MS 200     // back off the object before the U-turn

/*
 * 1 = calibration mode, no wire needed: motors stay off and the OLED shows the 5
 * normalised readings and the on / off pattern, refreshed every LOG_READINGS_MS.
 * Move the bar over white, black and pink and note the numbers. Set back to 0 to run.
 */
#define CALIBRATE_LSA 0

/*
 * 1 = print the 5 normalised readings every LOG_READINGS_MS on the serial monitor while running
 */
#define LOG_READINGS 0
#define LOG_READINGS_MS 200

#endif
