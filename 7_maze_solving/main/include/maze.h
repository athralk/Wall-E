#ifndef MAZE_H
#define MAZE_H

/*
 * Shared pieces of the maze state machine: the list of states, the context
 * struct every state handler works on, and the handler prototypes.
 *
 *   FOLLOW    state_follow.c    PID line following, spots everything else
 *   APPROACH  state_junction.c  drive onto a junction, pick an exit
 *   TURN      state_turns.c     every turn in place: 90 degree, 45 degree, acute, U-turn
 *   SEARCH    state_turns.c     sweep to find a lost line
 *   DROP      drop_zone.c       pink zone: stop, back off, then U-turn
 *   DONE      state_turns.c     end zone, stopped (resumes if put back on a line)
 *
 * The colour blind section has no state of its own: colour_blind.c flips the
 * sensor polarity and FOLLOW keeps going as normal.
 */

#include <stdbool.h>
#include <stdint.h>
#include "line_sensor.h"
#include "junction.h"

/*
 * Values are what the 4 bar graph LEDs show for each state
 */
typedef enum
{
    STATE_FOLLOW = 0x1,
    STATE_APPROACH = 0x2,
    STATE_TURN = 0x4,
    STATE_SEARCH = 0x8,
    STATE_DROP = 0x9,
    STATE_DONE = 0xF,
} maze_state_t;

/*
 * What a TURN is for. All of them pivot in place until the centre sensor lands on a
 * line, they only differ in how long the line is ignored at the start and the timeout.
 */
typedef enum
{
    TURN_PIVOT,     // 90 degree exit at a junction
    TURN_SHARP,     // 45 degree bend, or an acute exit found by the side IR
    TURN_UTURN,     // dead end / after the drop zone, sets after_uturn
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
    int sharp_dir, sharp_count;     // confirm() counters for 45 degree turns
    int lost_count;                 // loops in a row with no line under the bar
    int centred_count;              // readings in a row with the line in the middle of the bar
    uint32_t lost_since_ms;         // when the line vanished (0 = it hasn't)
    bool last_centred;              // line was in the middle of the bar the last time we saw it
    uint32_t hold_until_ms;         // start / colour flip: skip junction / sharp / dead end checks until then
    int rearm_count;                // after a junction / turn: plain line readings still needed
                                    // before the next turn can be detected (REARM_LOOPS)
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
 * Time spent in the current state
 */
uint32_t state_elapsed(const maze_ctx_t *ctx);

/*
 * Switches state: logs it, restarts the state timer and updates the bar graph
 */
void set_state(maze_ctx_t *ctx, maze_state_t next);

/*
 * Starting a state: sets it and the motors in one go (state_turns.c)
 */
void start_turn(maze_ctx_t *ctx, turn_kind_t kind, int dir);
void start_search(maze_ctx_t *ctx, int dir);
void back_to_follow(maze_ctx_t *ctx);

/*
 * One handler per state, called every loop while in that state
 */
void handle_follow(maze_ctx_t *ctx);
void handle_approach(maze_ctx_t *ctx);
void handle_turn(maze_ctx_t *ctx);
void handle_search(maze_ctx_t *ctx);
void handle_drop(maze_ctx_t *ctx);
void handle_done(maze_ctx_t *ctx);

#endif
