#ifndef MAZE_H
#define MAZE_H

/*
 * Shared pieces of the maze state machine: the list of states, the context
 * struct every state handler works on, and the handler prototypes.
 *
 *   FOLLOW    state_follow.c    PID line following (smooth curves, 45 degree bends), spots
 *                               junctions, acute exits and dead ends
 *   APPROACH  state_junction.c  drive onto a junction, pick an exit
 *   TURN_90   turn_90.c         90 degree pivot onto a junction exit / corner, or U-turn
 *   TURN_120  turn_120.c        pivot onto an acute (~120 degree) exit
 *   SEARCH    state_turns.c     sweep to find a lost line
 *   OBJECT_DETECTED state_turns.c  front IR object + all-white LSA; starts a U-turn
 *   DONE      state_turns.c     end zone, stopped (resumes if put back on a line)
 *
 * Pink drop zone (drop_zone.c) is switched off. Colour blind (colour_blind.c) runs
 * in FOLLOW and APPROACH so the black-on-white section can be tracked.
 */

#include <stdbool.h>
#include <stdint.h>
#include "line_sensor.h"
#include "junction.h"

/*
 * State ids (also printed in the log on every state change)
 */
typedef enum
{
    STATE_FOLLOW = 0x1,
    STATE_APPROACH = 0x2,
    STATE_TURN_90 = 0x4,
    STATE_TURN_120 = 0x6,
    STATE_SEARCH = 0x8,
    STATE_DROP = 0x9,           // pink zone, switched off
    STATE_OBJECT_DETECTED = 0xA,
    STATE_DONE = 0xF,
} maze_state_t;

/*
 * What a TURN_90 is for
 */
typedef enum
{
    TURN_PIVOT,     // 90 degree exit at a junction or corner
    TURN_UTURN,     // dead end, sets after_uturn
} turn_kind_t;

typedef struct
{
    maze_state_t state;
    uint32_t state_start_ms;    // when the current state started, for the timeouts
    lsa_state_t lsa;            // this loop's sensor reading

    // FOLLOW
    int node_confirm;               // side branches: loops in a row with a branch under the bar
    int hits_left, hits_right;      // loops each side was seen as a branch at this junction (FOLLOW + APPROACH)
    line_width_t line_width;        // normal width of the line, learned while following
    int sharp_dir, sharp_count;     // only used by colour_blind.c (off)
    int lost_count;                 // loops in a row with no line under the bar
    int centred_count;              // readings in a row with the line in the middle of the bar
    uint32_t lost_since_ms;         // when the line vanished (0 = it hasn't)
    int line_side;                  // where the line was last seen off centre: -1 left, +1 right, 0 not yet
    int object_confirm_count;       // consecutive loops with front IR active and all five LSA sensors white
    bool last_centred;              // line was in the middle of the bar the last time we saw it
    uint32_t hold_until_ms;         // start / colour flip: skip junction / sharp / dead end checks until then
    int rearm_count;                // after a junction / turn: plain line readings still needed
                                    // before the next turn can be detected (REARM_LOOPS)
    bool leaving_start;             // just started on a white patch: drive straight out of it
    bool seen_line;                 // line found since power on / resume (ends the start hold)
    uint32_t kick_until_ms;         // after a stop: drive harder until then so the wheels get going
    uint32_t turn_end_ms;           // when the last turn / approach handed back to FOLLOW (IR re-arm)
    uint32_t side_left_ms;          // last time a second line was seen left of the main line (A0 or side IR, 0 = never)
    uint32_t side_right_ms;         // same on the right (A4)
    bool ir_prev;                   // side IR reading last loop, to log only when it first sees a line

    // Turns
    turn_kind_t turn_kind;          // what the current TURN is for
    int turn_dir;                   // direction of the current turn / search, -1 left, +1 right
    bool left_old_line;             // TURN: centre sensor has come off the line we turned from
    bool turn_slowed;               // TURN: start kick over, now at TURN_DUTY
    bool turn_near;                 // TURN: a sensor on the turn side sees the new line, easing in at TURN_SLOW_DUTY
    uint32_t search_timeout;        // SEARCH: how long before sweeping the other way
    int32_t search_net_ms;          // SEARCH: time spun right minus time spun left (net rotation)
    uint32_t pivot_ms;              // how long the last 90 degree pivot took, to judge SEARCH rotation
    bool ir_seen;                   // side IR saw a left branch since the junction was first spotted (log only)

    // Recovery
    uint32_t line_seen_since_ms;    // DONE: when the line appeared under the bar (0 = not seen)

    // Progress through the maze
    int node_count;                 // junctions passed so far, "N: x" on the OLED
    bool after_uturn;               // last thing we did was a U-turn
    bool drop_done;                 // pink zone already handled, pink checks are off
    bool display_dirty;             // redraw the OLED on the next display_update()
} maze_ctx_t;

uint32_t now_ms(void);

/*
 * Name of the current state, for the live LSA web page
 */
const char *maze_state_text(void);

/*
 * Time spent in the current state
 */
uint32_t state_elapsed(const maze_ctx_t *ctx);

/*
 * Switches state: logs it and restarts the state timer
 */
void set_state(maze_ctx_t *ctx, maze_state_t next);

/*
 * Starting a state: sets it and the motors in one go (state_turns.c)
 */
void start_search(maze_ctx_t *ctx, int dir);
void back_to_follow(maze_ctx_t *ctx);

/*
 * Shared by turn_90.c and turn_120.c (state_turns.c)
 */
void finish_turn(maze_ctx_t *ctx);

/*
 * One handler per state, called every loop while in that state
 */
void handle_follow(maze_ctx_t *ctx);
void handle_approach(maze_ctx_t *ctx);
void handle_object_detected(maze_ctx_t *ctx);
void handle_search(maze_ctx_t *ctx);
void handle_done(maze_ctx_t *ctx);

#endif
