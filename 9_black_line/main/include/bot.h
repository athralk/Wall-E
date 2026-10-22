#ifndef BOT_H
#define BOT_H

/*
 * The bot's states and the one struct they all share.
 *
 *   WAIT      motors off until the bar sits on a line (start, after a failed turn)
 *   FOLLOW    PID on the line centre (or its left edge, see EDGE_AFTER_MS)  follow.c
 *   APPROACH  edge mode, left branch seen: drive onto the junction          follow.c
 *   TURN      pivot until the line is back under the middle of the bar      turns.c
 *   DONE      end zone, motors off (put it back on a line to go on)
 */

#include <stdbool.h>
#include <stdint.h>
#include "line_sensor.h"

/*
 * Values are what the bar graph LEDs show
 */
typedef enum
{
    ST_FOLLOW = 0x1,
    ST_APPROACH = 0x2,
    ST_TURN = 0x4,
    ST_WAIT = 0x8,
    ST_DONE = 0xF,
} bot_state_t;

typedef struct
{
    bot_state_t state;
    uint32_t state_ms;          // when the current state started
    uint32_t run_ms;            // when the run started (for EDGE_AFTER_MS)
    lsa_state_t lsa;            // this loop's sensor reading

    uint32_t kick_until;        // FOLLOW: pull away harder until then
    int lost_count;             // FOLLOW: loops in a row with no line
    uint32_t creep_until;       // FOLLOW: rolling on to a corner until then (0 = not)
    uint32_t left_seen;         // FOLLOW centre: last time A0 saw the line
    uint32_t right_seen;        // FOLLOW centre: last time A4 saw the line
    int last_first;             // FOLLOW edge: first white sensor the last time there was white
    int left_count;             // FOLLOW edge: loops in a row with a left branch
    bool armed;                 // FOLLOW edge: edge found since the last turn
    uint32_t white_since;       // FOLLOW: when the whole bar went white (0 = it isn't)
    bool edge_was;              // FOLLOW: was in edge mode last loop (to log the switch)

    int turn_dir;               // TURN: -1 left, +1 right
    bool a0_black, crossed;     // TURN edge left: A0 was black, then the new line came in
    bool slowed, easing;        // TURN: past the start kick / easing onto the line
    bool uturn;                 // TURN: no line within ~90 degrees, going round
    bool tried_back;            // TURN: timed out once and turned the other way
    uint32_t turn90_ms;         // how long the last ~90 degree turn took

    uint32_t line_since;        // WAIT / DONE: when a line appeared under the bar (0 = none)
    int nodes, uturns;          // counted for the log and the OLED
    bool dirty;                 // OLED needs a redraw
} bot_t;

uint32_t now_ms(void);
uint32_t elapsed(const bot_t *b);
void set_state(bot_t *b, bot_state_t next);
bool edge_mode(const bot_t *b);

void start_follow(bot_t *b);
void follow(bot_t *b);
void approach(bot_t *b);
void start_turn(bot_t *b, int dir);
void turn(bot_t *b);
void wait_for_line(bot_t *b);

#endif
