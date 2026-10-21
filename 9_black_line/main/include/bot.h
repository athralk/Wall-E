#ifndef BOT_H
#define BOT_H

/*
 * The bot's states and the one struct they all share.
 *
 *   WAIT      motors off until the bar sits on a line (start, after a failed turn)
 *   FOLLOW    PID on the left edge of the white line                  follow.c
 *   APPROACH  left branch seen: drive onto the junction, then turn    follow.c
 *   TURN      pivot left or right until the edge is back under A1|A2  turns.c
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
    lsa_state_t lsa;            // this loop's sensor reading

    int left_count;             // FOLLOW: loops in a row with a left branch
    int lost_count;             // FOLLOW: loops in a row with no white
    uint32_t kick_until;        // FOLLOW: pull away harder until then
    bool armed;                 // FOLLOW: edge found since the last turn, left branches count
    uint32_t white_since;       // FOLLOW: when the whole bar went white (0 = it isn't)

    int turn_dir;               // TURN: -1 left, +1 right
    bool a0_black;              // TURN: A0 has been on black during this turn
    bool crossed;               // TURN left: the new line has come in under A0
    bool slowed, easing;        // TURN: past the start kick / easing onto the line
    bool uturn;                 // TURN right: no right turn within 90 degrees, going round
    uint32_t turn90_ms;         // how long the last left turn (about 90 degrees) took

    uint32_t line_since;        // WAIT / DONE: when a line appeared under the bar (0 = none)
    int nodes, uturns;          // counted for the log and the OLED
    bool dirty;                 // OLED needs a redraw
} bot_t;

uint32_t now_ms(void);
uint32_t elapsed(const bot_t *b);
void set_state(bot_t *b, bot_state_t next);

void start_follow(bot_t *b);
void follow(bot_t *b);
void approach(bot_t *b);
void start_turn(bot_t *b, int dir);
void turn(bot_t *b);
void wait_for_line(bot_t *b);

#endif
