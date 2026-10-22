/*
 * TURN and WAIT / DONE.
 *
 * A turn pivots in place, straight out of following and straight back into it (no stops):
 * a short kick, then TURN_DUTY, easing to TURN_SLOW_DUTY once a sensor on the turn side
 * sees the new line, and back to FOLLOW TURN_STOP_EARLY sensors before it is centred.
 *   Centre mode: done when the centre sensor (or the one before it) is on the line.
 *   Edge mode:   done when the left edge is back at A1|A2. Left turns must first see the
 *                new line come in under A0 (the bar may start on the line ahead).
 * A right turn with no line within ~90 degrees is a dead end: it keeps going round onto
 * the other side of the line (the U-turn). Nothing at all: try the other way once.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "bot.h"
#include "config.h"
#include "edge.h"
#include "motion.h"

static const char *TAG = "bot";

void start_turn(bot_t *b, int dir)
{
    b->turn_dir = dir;
    b->a0_black = b->crossed = false;
    b->slowed = b->easing = false;
    b->uturn = b->tried_back = false;
    ESP_LOGI(TAG, "turn %s", dir < 0 ? "left" : "right");
    set_state(b, ST_TURN);
    pivot(dir, TURN_KICK_DUTY);
}

static bool turn_done(const bot_t *b)
{
    const bool *on = b->lsa.on_line;
    int first = edge_first_white(&b->lsa);
    if (!edge_mode(b))
    {
        bool inner = b->turn_dir < 0 ? on[1] : on[3];
        return on[2] || (TURN_STOP_EARLY > 0 && inner);
    }
    if (b->turn_dir > 0)    // white comes in from the right: first white 4, 3, 2
        return first <= EDGE_TARGET + TURN_STOP_EARLY && !on[0];
    // new line comes in under A0 and moves right: first white 0, 1, 2
    return b->crossed && first >= EDGE_TARGET - TURN_STOP_EARLY && first <= EDGE_TARGET;
}

/*
 * Nothing found in TURN_TIMEOUT_MS: turn back the other way once, then give up and wait
 */
static void turn_timeout(bot_t *b)
{
    if (!b->tried_back)
    {
        ESP_LOGW(TAG, "no line, trying the other way");
        b->tried_back = true;
        b->turn_dir = -b->turn_dir;
        b->slowed = b->easing = false;
        b->state_ms = now_ms();
        pivot(b->turn_dir, TURN_KICK_DUTY);
        return;
    }
    stop();
    ESP_LOGW(TAG, "turn found no line, put me back on one");
    b->line_since = 0;
    set_state(b, ST_WAIT);
}

void turn(bot_t *b)
{
    const bool *on = b->lsa.on_line;
    uint32_t e = elapsed(b);
    int dir = b->turn_dir;

    if (!on[0])
        b->a0_black = true;
    if (dir < 0 && b->a0_black && on[0])
        b->crossed = true;

    // Speed: kick, then TURN_DUTY, then ease once the new line reaches the turn side
    bool near = dir < 0 ? (on[0] || on[1]) : (on[3] || on[4]);
    if (!b->slowed && e >= TURN_KICK_MS)
    {
        b->slowed = true;
        pivot(dir, TURN_DUTY);
    }
    if (b->slowed && near && !b->easing)
    {
        b->easing = true;
        pivot(dir, TURN_SLOW_DUTY);
    }

    if (turn_done(b))
    {
        if (BRAKE_MS > 0)
        {
            pivot(-dir, BRAKE_DUTY);
            vTaskDelay(BRAKE_MS / portTICK_PERIOD_MS);
        }
        if (dir < 0 && !b->uturn && !b->tried_back)
            b->turn90_ms = e;   // a left turn is ~90 degrees: the dead end check uses it
        start_follow(b);
        return;
    }

    // Right turn, nothing within ~90 degrees: no right turn here, it's a dead end
    uint32_t t90 = b->turn90_ms ? b->turn90_ms : RIGHT_90_MS;
    if (dir > 0 && !b->uturn && e >= t90 * RIGHT_CHECK_PCT / 100)
    {
        b->uturn = true;
        b->uturns++;
        b->dirty = true;
        ESP_LOGI(TAG, "no line within 90 degrees: dead end, U-turn");
    }

    if (e >= TURN_TIMEOUT_MS)
        turn_timeout(b);
}

void wait_for_line(bot_t *b)
{
    stop();
    // A line = some white but not all of it (all white is the end zone)
    bool line = b->lsa.count > 0 && b->lsa.count < LSA_COUNT;
    if (!line)
    {
        b->line_since = 0;
        return;
    }
    if (b->line_since == 0)
        b->line_since = now_ms();
    if (now_ms() - b->line_since >= START_ON_LINE_MS)
    {
        ESP_LOGI(TAG, "on a line, go");
        if (b->run_ms == 0)
            b->run_ms = now_ms();
        start_follow(b);
        b->kick_until = now_ms() + KICK_MS;     // pulling away from standing still
    }
}
