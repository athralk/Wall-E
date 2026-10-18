/*
 * FOLLOW state: normal PID line following.
 *
 * Every loop it checks, in this order, and hands off to another state if one fires:
 *   0. pink drop zone          -> DROP     (drop_zone.c)
 *   1. side branch (junction)  -> APPROACH (state_junction.c)
 *   2. 45 degree turn          -> SHARP
 *   3. line gone               -> UTURN if it was centred (dead end), else SEARCH
 *   4. none of the above: PID
 *
 * Checks 1-3 are skipped (bot drives straight if it sees no line) for START_HOLD_MS
 * at power on and INVERT_HOLD_MS after a colour flip, so the start and the edges of
 * the colour blind block can't look like a junction or a dead end.
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "pid.h"
#include "junction.h"
#include "drop_zone.h"
#include "display.h"

static const char *TAG = "maze";

void handle_follow(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;
    display_update(ctx);

    // 0. Pink: once confirmed go to DROP. While not sure yet, drive straight so the
    //    line-lost check can't U-turn us before the confirm count is reached.
    drop_check_t pink = drop_zone_check(ctx);
    if (pink == DROP_CONFIRMED)
    {
        start_drop(ctx);
        return;
    }
    if (pink == DROP_MAYBE)
    {
        drive(BASE_DUTY, BASE_DUTY);
        return;
    }

    bool hold = now_ms() < ctx->hold_until_ms;

    if (!hold)
    {
        // 1. Side branch held for CORNER_CONFIRM loops: drive onto the junction
        int branches = detect_branches(lsa);
        if (confirm(branches ? 1 : 0, &ctx->node_seen, &ctx->node_confirm) >= CORNER_CONFIRM)
        {
            ctx->node_seen = ctx->node_confirm = 0;
            ctx->seen_branches = branches;
            set_state(ctx, STATE_APPROACH);
            drive(APPROACH_DUTY, APPROACH_DUTY);
            return;
        }

        // 2. Line has slid out to an outer sensor: 45 degree turn
        if (confirm(detect_sharp(lsa), &ctx->sharp_dir, &ctx->sharp_count) >= SHARP_CONFIRM)
        {
            int dir = ctx->sharp_dir;
            ctx->sharp_dir = ctx->sharp_count = 0;
            ESP_LOGI(TAG, "sharp %s turn", dir < 0 ? "left" : "right");
            start_sharp(ctx, dir);
            return;
        }
    }

    // 3. No line at all: dead end or overshoot, decided by where the line was last seen
    if (lsa->count == 0)
    {
        if (hold)
        {
            // Start of the run / just after a colour flip: drive straight onto the line
            drive(BASE_DUTY, BASE_DUTY);
            return;
        }

        ctx->lost_count++;
        if (ctx->last_centred)
        {
            // Line ended while it was under the middle of the bar: dead end, but only once
            // it has stayed gone for DEADEND_CONFIRM loops of driving straight, so a thin
            // line slipping between two sensors isn't taken as one
            drive(BASE_DUTY, BASE_DUTY);
            if (ctx->lost_count >= DEADEND_CONFIRM)
            {
                ctx->lost_count = 0;
                ESP_LOGI(TAG, "dead end, U-turn");
                start_uturn(ctx);
            }
        }
        else if (ctx->lost_count >= LOST_CONFIRM)
        {
            // Line slipped off one side: search toward the side it was last on
            ctx->lost_count = 0;
            start_search(ctx, pid_last_error() >= 0 ? 1 : -1);
        }
        // else brief gap: keep the last motor output going
        return;
    }
    ctx->lost_count = 0;
    // Remember if the line is near the middle, used for the dead end check above
    ctx->last_centred = !lsa->on_line[0] && !lsa->on_line[4];

    // 4. Normal PID following: speed up one wheel and slow the other
    float correction = pid_compute(lsa->position);
    drive(bound(BASE_DUTY + correction, MIN_DUTY, MAX_DUTY),
          bound(BASE_DUTY - correction, MIN_DUTY, MAX_DUTY));
}
