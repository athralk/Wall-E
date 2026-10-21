/*
 * FOLLOW and APPROACH.
 *
 * FOLLOW rides the left edge of the white line with the PID. Two things end it:
 *   - white under A0 and A1 (always black here): a line from the left -> APPROACH
 *   - no white at all: right corner, dead end or drifted left -> turn right
 * Right branches never show: the left edge just carries on past them.
 * Together that is the left hand rule: left, else straight, else right, else U-turn.
 */
#include "sra_board.h"
#include "bot.h"
#include "config.h"
#include "edge.h"
#include "motion.h"
#include "pid.h"

static const char *TAG = "bot";

void start_follow(bot_t *b)
{
    pid_reset();
    b->left_count = 0;
    b->lost_count = 0;
    b->armed = false;
    b->white_since = 0;
    b->kick_until = now_ms() + KICK_MS;
    set_state(b, ST_FOLLOW);
}

void follow(bot_t *b)
{
    const lsa_state_t *lsa = &b->lsa;
    uint32_t t = now_ms();

    // Armed once the edge is properly under the bar again (A0 A1 black, some white). Until
    // then all white is just the bot sitting on the line, and the PID steers it left onto
    // the edge (no junction from it).
    if (!lsa->on_line[0] && !lsa->on_line[1] && lsa->count > 0)
        b->armed = true;

    // Whole bar white for END_WHITE_MS (still steering left): the end zone
    if (lsa->count == LSA_COUNT)
    {
        if (b->white_since == 0)
            b->white_since = t;
        if (t - b->white_since >= END_WHITE_MS)
        {
            stop();
            ESP_LOGI(TAG, "END zone: %d nodes, %d U-turns", b->nodes, b->uturns);
            b->line_since = 0;
            set_state(b, ST_DONE);
            return;
        }
    }
    else
    {
        b->white_since = 0;
    }

    // 1. Line from the left, held LEFT_CONFIRM loops (only once armed)
    bool left = b->armed && edge_left_branch(lsa);
    b->left_count = left ? b->left_count + 1 : 0;
    if (b->left_count >= LEFT_CONFIRM)
    {
        set_state(b, ST_APPROACH);
        drive(APPROACH_DUTY, APPROACH_DUTY);
        return;
    }
    if (b->left_count > 0)
    {
        // Maybe a branch: drive straight, don't let the PID swing into it yet
        drive(BASE_DUTY, BASE_DUTY);
        return;
    }

    // 2. No white (00111 -> 00000): pivot right in place. Right turn, or U-turn if none.
    if (lsa->count == 0)
    {
        if (++b->lost_count >= LOST_CONFIRM)
            start_turn(b, 1);
        return;     // one bad reading: keep the last motor output
    }
    b->lost_count = 0;

    // 3. PID on the edge. Just after a stop, push harder so the wheels get going.
    bool kick = t < b->kick_until;
    int base = kick ? KICK_DUTY : BASE_DUTY;
    int low = kick ? KICK_MIN_DUTY : MIN_DUTY;
    float c = pid_compute(edge_error(lsa));
    drive(bound(base + c, low, MAX_DUTY), bound(base - c, low, MAX_DUTY));
}

void approach(bot_t *b)
{
    uint32_t e = elapsed(b);
    if (e < APPROACH_MS)
        return;

    // Still all white on the junction: a wide white patch, not a turn. Steer left onto
    // its left edge (FOLLOW, not armed). If it never ends, FOLLOW stops at the end zone.
    if (b->lsa.count == LSA_COUNT)
    {
        ESP_LOGI(TAG, "all white: steering onto the left edge");
        start_follow(b);
        return;
    }

    // Line still ahead too = a node (left + straight), else just a left corner
    if (b->lsa.count > 0)
    {
        b->nodes++;
        b->dirty = true;
        ESP_LOGI(TAG, "node %d: left and straight, going left", b->nodes);
    }
    start_turn(b, -1);
}
