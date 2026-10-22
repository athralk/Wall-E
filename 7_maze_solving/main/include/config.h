#ifndef CONFIG_H
#define CONFIG_H

/*
 * Every tunable number for the maze in one place. This is the only file you
 * should need to edit while tuning on the track.
 *
 * Duties are motor duty cycles (motion.c caps each wheel at 50).
 * Times are in milliseconds, "loops" are runs of the control loop (LOOP_MS each).
 */

#define LOOP_MS 10              // control loop period. Not below 10: the system tick is 10 ms, a
                                // shorter pause rounds to 0 and the loop never lets CPU 0 rest
                                // (task watchdog, Wi-Fi and the OLED starved)
#define PID_REAL_DT 0           // 1 = PID derivative uses the measured loop time (micromouse style), 0 = per loop as before

/*
 * Line following speeds (FOLLOW state)
 */
#define BASE_DUTY 28
#define MIN_DUTY 25             // slowest a wheel goes while steering (25 and below can stall)
#define MAX_DUTY 38             // fastest a wheel goes while following (steering). Turns have their own speeds

#define NODE_MEMORY 0           // 1 = straight first remembers nodes and comes back to untried exits.
                                // 0 = plain rule: straight first, after a U-turn the turn first

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
#define CORNER_CONFIRM 2        // loops in a row a side branch must be seen before we treat it as a corner / node.
                                // Each side must also have been seen this many loops to count as an exit
#define LINE_WIDTH_HISTORY 64   // plain readings remembered to learn the line width (the widest of them is used)
#define LINE_WIDTH_MIN_SAMPLES 8 // fewer than this (start, colour flip): assume the widest plain line (3)

/*
 * Side IR sensor (MH-B module), facing down, level with the LSA but out to the LEFT of it.
 * The main line never passes under it, so it only sees the line when a LEFT branch does:
 * that counts as a left branch straight away (no CORNER_CONFIRM) even if the LSA pattern missed it. Right branches still
 * come from the LSA alone. Tested with 8_demo_ir.
 */
#define IR_ENABLED 0            // 1 = use the side IR. Off: LSA only
#define IR_PIN 27               // OUT pin, J14 header pin 12
#define IR_WHITE_LEVEL 0        // OUT level over white (the module pulls OUT low on a bright surface)
// The pin is also watched by a high priority interrupt, so a branch that passes under the IR
// between two loops (the IR spot is small) is still caught
#define IR_REARM_MS 120         // after a turn the IR is ignored only this long (the LSA waits REARM_LOOPS
                                // plain readings, which can be much longer and used to hide real branches)
#define APPROACH_DUTY 40        // straight drive to bring the axle over the corner
#define APPROACH_MS 130         // depends on sensor bar to axle distance, tune on track
#define TURN_DUTY 50            // steady pivot speed for every turn (50 = the max motion.c allows)
// #define TURN_KICK_DUTY 50       // every turn starts at this (max motion.c allows) to break static friction,   (not used: turns run at a steady TURN_DUTY)
// #define TURN_KICK_MS 80         // for this long, then drops to TURN_DUTY. Raise TURN_DUTY if turns stall half way   (not used: turns run at a steady TURN_DUTY)
// #define TURN_SLOW_DUTY 45       // once a sensor on the turn side sees the new line, finish the turn at this   (not used: turns run at a steady TURN_DUTY)
                                // (was 40: below 45 the wheels stall mid turn)
                                // speed so it eases onto the line instead of slamming past it
#define PIVOT_MIN_MS 40         // ignore the line for this long after starting a pivot
#define PIVOT_TIMEOUT_MS 1500   // give up and search if the new line never shows
#define BRAKE_DUTY 40           // short reverse pulse at the end of a turn, kills overshoot
#define BRAKE_MS 40
#define REARM_LOOPS 5           // plain line readings needed after a turn before the next turn can trigger
#define END_CONFIRM_MS 200      // end zone: bar must stay all white this much longer (still driving) before stopping

/*
 * 45 degree bends and curves: no turn state, the PID line following steers round them
 */

/*
 * Acute (~120 degree) exits: the exit arm runs back under the outer sensor on its side (A0 / A4)
 * while the centre is still on the main line, just before the main line ends. Line lost this
 * soon after that: pivot to that side (left first) instead of U-turning / searching.
 */
#define ACUTE_WINDOW_MS 400
#define ACUTE_TIMEOUT_MS 2500   // turn_120: no line in this long -> SEARCH

/*
 * After a turn / at the start, for KICK_MS FOLLOW uses KICK_DUTY as the base and keeps both
 * wheels at KICK_MIN_DUTY or more, so the wheels never drop to a speed where they stall
 */
#define KICK_DUTY 45
#define KICK_MIN_DUTY 38
#define KICK_MS 250

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
 * Lost line recovery (gaps, acute turns sharper than 90)
 */
#define LOST_CONFIRM 3          // loops with no line before turning toward where it went
#define DEADEND_MS 100          // line gone this long (driving straight) before a dead end U-turn
#define CENTRED_CONFIRM 3       // dead end only if the line sat in the middle this many readings
                                // in a row before it vanished (else it was a missed turn: search)
#define START_HOLD_MS 1000      // (start now waits for the line, see START_ON_LINE_MS)
#define START_ON_LINE_MS 500    // at power on / resume: motors stay off until the bar has seen a line this long
#define START_PUSH_DUTY 50      // then both wheels this hard...
#define START_PUSH_MS 120       // ...for this long, to break free from standing still
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
 * Colour blind section: black line on a white block. Raw 10001 (white outside, black middle)
 * switches to black line following, raw 01110 (white middle, black outside) switches back.
 */
#define INVERT_CONFIRM 6        // loops of raw 10001 (white outside, black middle) before black line following
#define INVERT_EXIT_CONFIRM 5   // loops of raw 01110 (white middle, black outside) before going back to white line
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

#define DISPLAY_MS 200          // how often the display task checks if the OLED needs a redraw

#endif
