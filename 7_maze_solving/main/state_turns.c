/*
 * Shared turn pieces (finish_turn, back_to_follow) and the other states:
 * the turns themselves are in turn_90.c and turn_120.c.
 *
 *   SEARCH  last resort sweep, wider each time
 *   DONE    end zone, stay stopped
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "pid.h"

static const char *TAG = "maze";

void back_to_follow(maze_ctx_t *ctx)
{
    pid_reset();
    ctx->kick_until_ms = now_ms() + KICK_MS;
    ctx->turn_end_ms = now_ms();
    ctx->last_centred = true;
    ctx->centred_count = 0;
    ctx->lost_count = 0;
    ctx->lost_since_ms = 0;
    ctx->rearm_count = REARM_LOOPS;
    set_state(ctx, STATE_FOLLOW);
}

/*
 * End of a turn: optional short reverse pulse (BRAKE_MS), then straight into line following
 * with the wheels already driving forward at KICK_DUTY. No stop: pulling away from a
 * standstill at line following speed is what stalled the motors after turns.
 */
void finish_turn(maze_ctx_t *ctx)
{
    if (BRAKE_MS > 0)
    {
        pivot(-ctx->turn_dir, BRAKE_DUTY);
        vTaskDelay(BRAKE_MS / portTICK_PERIOD_MS);
    }
    drive(KICK_DUTY, KICK_DUTY);
    back_to_follow(ctx);
}

void start_search(maze_ctx_t *ctx, int dir)
{
    ctx->turn_dir = dir;
    ctx->search_timeout = SEARCH_TIMEOUT_MS;
    ctx->search_net_ms = 0;
    set_state(ctx, STATE_SEARCH);
    pivot(dir, SEARCH_DUTY);
}

/*
 * True if SEARCH has spun the bot round by about 135 degrees or more, i.e. it found the
 * line behind it. Judged by time, using how long a 90 degree pivot took (TURN_DUTY and
 * SEARCH_DUTY are close), or SEARCH_UTURN_MS before the first pivot.
 */
static bool search_turned_around(const maze_ctx_t *ctx)
{
    uint32_t limit = ctx->pivot_ms ? ctx->pivot_ms * 3 / 2 : SEARCH_UTURN_MS;
    int32_t net = ctx->search_net_ms < 0 ? -ctx->search_net_ms : ctx->search_net_ms;
    return (uint32_t)net >= limit;
}

/*
 * SEARCH: pivot one way, then back the other way for twice as long,
 * until any of the middle three sensors sees the line
 */
void handle_search(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;
    if (lsa->on_line[1] || lsa->on_line[2] || lsa->on_line[3])
    {
        stop();
        ctx->search_net_ms += ctx->turn_dir * (int32_t)state_elapsed(ctx);
        if (search_turned_around(ctx))
        {
            // The line was behind us: this was a dead end with the line slightly off
            // centre, so count it as a U-turn (the node memory needs to know)
            ESP_LOGI(TAG, "search turned round (%ld ms net), treating as dead end U-turn",
                     (long)ctx->search_net_ms);
            ctx->after_uturn = true;
        }
        back_to_follow(ctx);
        return;
    }

    if (state_elapsed(ctx) >= ctx->search_timeout)
    {
        // Sweep back the other way, twice as far so we cover the first side again
        ctx->search_net_ms += ctx->turn_dir * (int32_t)state_elapsed(ctx);
        ctx->turn_dir = -ctx->turn_dir;
        ctx->search_timeout *= 2;
        ctx->state_start_ms = now_ms();
        ESP_LOGW(TAG, "search flip, now %s", ctx->turn_dir < 0 ? "left" : "right");
        pivot(ctx->turn_dir, SEARCH_DUTY);
    }
}

/*
 * DONE: end zone, stay stopped. If it stopped by mistake, put the bot back on a line:
 * once the line has been under the bar (and the bar isn't all white) for RESUME_MS,
 * it carries on as if just switched on.
 */
void handle_done(maze_ctx_t *ctx)
{
    stop();

    bool on_a_line = ctx->lsa.count > 0 && ctx->lsa.raw_white_count < LSA_COUNT;
    if (!on_a_line)
    {
        ctx->line_seen_since_ms = 0;
        return;
    }
    if (ctx->line_seen_since_ms == 0)
    {
        ctx->line_seen_since_ms = now_ms();
        return;
    }
    if (now_ms() - ctx->line_seen_since_ms >= RESUME_MS)
    {
        ESP_LOGI(TAG, "put back on a line, resuming");
        ctx->line_seen_since_ms = 0;
        ctx->seen_line = false;     // FOLLOW waits START_ON_LINE_MS again before moving
        ctx->hold_until_ms = now_ms() + START_HOLD_MS;
        back_to_follow(ctx);
    }
}
