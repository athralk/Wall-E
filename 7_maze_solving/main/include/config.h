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

/*
 * Line following speeds (FOLLOW state)
 */
#define BASE_DUTY 40
#define MIN_DUTY 30
#define MAX_DUTY 55

/*
 * 90 degree corners: line suddenly covers one side of the bar (11100 / 00111).
 * Drive forward until the axle is over the corner, then pivot in place onto the new line.
 */
#define CORNER_CONFIRM 2        // loops a side branch pattern must hold before we treat it as a corner / node
#define APPROACH_DUTY 30        // straight drive to bring the axle over the corner
#define APPROACH_MS 150         // depends on sensor bar to axle distance, tune on track
#define TURN_DUTY 55            // pivot speed
#define PIVOT_MIN_MS 40         // ignore the line for this long after starting a pivot
#define PIVOT_TIMEOUT_MS 1500   // give up and search if the new line never shows
#define BRAKE_DUTY 50           // short reverse pulse to kill pivot overshoot
#define BRAKE_MS 30
#define END_CONFIRM_MS 200      // end zone: bar must stay all white this much longer (still driving) before stopping

/*
 * 45 degree turns: the line slides out to the outermost sensor faster than PID can follow
 * (10000 / 11000). Swing hard toward it, inner wheel backward, until the centre finds it.
 */
#define SHARP_CONFIRM 2
#define SHARP_OUTER_DUTY 50     // wheel on the outside of the turn, forward
#define SHARP_INNER_DUTY 25     // wheel on the inside of the turn, backward
#define SHARP_TIMEOUT_MS 1000

/*
 * Nodes (junctions with 2+ exits). Straight always wins over a turn, except right after
 * a dead end U-turn, where straight would just take us back the way we came.
 */
#define TURN_PREFERENCE -1      // turn to take when there is no straight: -1 left, +1 right
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
#define UTURN_DIR TURN_PREFERENCE
#define UTURN_MIN_MS 400        // ignore the line for this long so we don't grab the stub we just left
#define UTURN_TIMEOUT_MS 3000

/*
 * Lost line recovery (gaps, acute turns sharper than 90)
 */
#define LOST_CONFIRM 3          // loops with no line before we start searching
#define DEADEND_CONFIRM 8       // loops with no line (driving straight) before a dead end U-turn
#define START_HOLD_MS 1000      // at power on: drive straight onto the line, no dead end / junction checks
#define SEARCH_DUTY 50
#define SEARCH_TIMEOUT_MS 800   // flip search direction after this, doubles each flip

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
#define PINK_MIN 600            // normalised reading (0 black .. 1000 white), PLACEHOLDER
#define PINK_MAX 800            // PLACEHOLDER
#define PINK_CONFIRM 5          // loops the middle three sensors must stay in the band
#define DROP_STOP_MS 500        // stand still so the object settles in the zone
#define DROP_REVERSE_DUTY 30
#define DROP_REVERSE_MS 200     // back off the object before the U-turn

/*
 * 1 = print the 5 normalised readings every LOG_READINGS_MS (for calibrating pink)
 */
#define LOG_READINGS 0
#define LOG_READINGS_MS 200

#endif
