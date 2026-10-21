/*
 * TURN and WAIT / DONE.
 *
 * A turn pivots in place until the edge is back under A1|A2 (00111):
 *   right: the new line comes in from the right side of the bar. If nothing shows within
 *          ~90 degrees it was a dead end, and the turn carries on round (~180 degrees) onto
 *          the other side of the same line: the U-turn.
 *   left:  the new line comes in under A0 and crosses the bar; stop once it has come in
 *          and its left edge sits at A1|A2 (the bar may start on the line ahead).
 * Start hard (TURN_KICK_DUTY), run at TURN_DUTY, ease to TURN_SLOW_DUTY near the line,
 * stop TURN_STOP_EARLY sensors before the edge and brake: momentum carries it the rest.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "bot.h"
#include "config.h"
#include "edge.h"
#include "motion.h"

static const char *TAG = "bot";

static void brake(int dir)
{
    pivot(-dir, BRAKE_DUTY);
    vTaskDelay(BRAKE_MS / portTICK_PERIOD_MS);
    stop();
}

void start_turn(bot_t *b, int dir)
{
    b->turn_dir = dir;
    b->crossed = false;
    b->a0_black = false;
    b->slowed = false;
    b->easing = false;
    b->uturn = false;
    ESP_LOGI(TAG, "turn %s", dir < 0 ? "left" : "right");
    set_state(b, ST_TURN);
    pivot(dir, TURN_KICK_DUTY);
}

/*
 * Turn is done once the edge has come (within TURN_STOP_EARLY sensors) to its place.
 * Right: white comes in from the right, first white sensor goes 4, 3, 2.
 * Left: the new line comes in under A0 and moves right, first white goes 0, 1, 2.
 */
static bool turn_done(const bot_t *b)
{
    int first = edge_first_white(&b->lsa);
    if (b->turn_dir > 0)
        return first <= EDGE_TARGET + TURN_STOP_EARLY && !b->lsa.on_line[0];
    return b->crossed && first >= EDGE_TARGET - TURN_STOP_EARLY && first <= EDGE_TARGET;
}

void turn(bot_t *b)
{
    const bool *on = b->lsa.on_line;
    uint32_t e = elapsed(b);
    int dir = b->turn_dir;

    // Left: the new line has come in once A0 turns white after being black during this turn
    // (A1 alone could still be the line ahead, if the bot sat a little right of its edge)
    if (!on[0])
        b->a0_black = true;
    if (dir < 0 && b->a0_black && on[0])
        b->crossed = true;

    // Speed: kick, then TURN_DUTY, then ease once the new line is close
    bool near = dir < 0 ? b->crossed : on[4];
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
        brake(dir);
        if (dir < 0)
            b->turn90_ms = e;   // a left turn is ~90 degrees: the right turn check uses it
        start_follow(b);
        return;
    }

    // Right: nothing within ~90 degrees, so no right turn here. Dead end: keep going round.
    uint32_t t90 = b->turn90_ms ? b->turn90_ms : RIGHT_90_MS;
    if (dir > 0 && !b->uturn && e >= t90 * RIGHT_CHECK_PCT / 100)
    {
        b->uturn = true;
        b->uturns++;
        b->dirty = true;
        ESP_LOGI(TAG, "no right turn within 90 degrees: dead end, U-turn");
    }

    if (e >= TURN_TIMEOUT_MS)
    {
        stop();
        ESP_LOGW(TAG, "turn found no line, put me back on one");
        b->line_since = 0;
        set_state(b, ST_WAIT);
    }
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
        start_follow(b);
    }
}
