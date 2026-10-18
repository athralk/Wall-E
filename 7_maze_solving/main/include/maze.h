#ifndef MAZE_H
#define MAZE_H

/*
 * Shared pieces of the maze state machine: the list of states, the context
 * struct every state handler works on, and the handler prototypes.
 *
 *   FOLLOW    state_follow.c    PID line following, spots everything else
 *   APPROACH  state_junction.c  drive onto a junction, pick an exit
 *   PIVOT     state_turns.c     90 degree turn in place
 *   SHARP     state_turns.c     45 degree swing
 *   UTURN     state_turns.c     180 degree spin (dead end, after the drop zone)
 *   SEARCH    state_turns.c     sweep to find a lost line
 *   DROP      drop_zone.c       pink zone: stop, back off, then UTURN
 *   DONE      state_turns.c     end zone, stopped
 *
 * The colour blind section has no state of its own: colour_blind.c flips the
 * sensor polarity and FOLLOW keeps going as normal.
 */

#include <stdbool.h>
#include <stdint.h>
#include "line_sensor.h"

/*
 * Values are what the 4 bar graph LEDs show for each state
 */
typedef enum
{
    STATE_FOLLOW = 0x1,
    STATE_APPROACH = 0x2,
    STATE_PIVOT = 0x4,
    STATE_SHARP = 0x6,
    STATE_SEARCH = 0x8,
    STATE_DROP = 0x9,
    STATE_UTURN = 0xC,
    STATE_DONE = 0xF,
} maze_state_t;

typedef struct
{
    maze_state_t state;
    uint32_t state_start_ms;    // when the current state started, for the timeouts
    lsa_state_t lsa;            // this loop's sensor reading

    // FOLLOW
    int node_seen, node_confirm;    // confirm() counters for side branches
    int sharp_dir, sharp_count;     // confirm() counters for 45 degree turns
    int lost_count;                 // loops in a row with no line under the bar
    bool last_centred;              // line was in the middle of the bar the last time we saw it
    uint32_t hold_until_ms;         // after a colour flip: skip junction / sharp / dead end checks until then

    // Turns
    int turn_dir;                   // direction of the current turn / search, -1 left, +1 right
    int seen_branches;              // side branches seen while driving over a junction
    bool left_old_line;             // pivot: centre sensor has come off the line we turned from
    uint32_t search_timeout;        // SEARCH: how long before sweeping the other way

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
void start_pivot(maze_ctx_t *ctx, int dir);
void start_sharp(maze_ctx_t *ctx, int dir);
void start_uturn(maze_ctx_t *ctx);
void start_search(maze_ctx_t *ctx, int dir);
void back_to_follow(maze_ctx_t *ctx);

/*
 * One handler per state, called every loop while in that state
 */
void handle_follow(maze_ctx_t *ctx);
void handle_approach(maze_ctx_t *ctx);
void handle_pivot(maze_ctx_t *ctx);
void handle_sharp(maze_ctx_t *ctx);
void handle_uturn(maze_ctx_t *ctx);
void handle_search(maze_ctx_t *ctx);
void handle_drop(maze_ctx_t *ctx);
void handle_done(maze_ctx_t *ctx);

#endif
