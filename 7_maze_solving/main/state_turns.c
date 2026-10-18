/*
 * The turning states. Each start_*() sets the state and the motors, each
 * handle_*() runs every loop and goes back to FOLLOW once the centre sensor
 * is on a line. Any turn that takes too long falls back to SEARCH.
 *
 *   PIVOT   90 degree turn in place at a junction
 *   SHARP   45 degree swing, outer wheel forward, inner wheel backward
 *   UTURN   180 degree spin at a dead end or after the drop zone
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
#include "display.h"

static const char *TAG = "maze";

/*
 * Ends a pivot / U-turn: short spin the other way to cancel momentum, stop,
 * and clear the PID so the old error doesn't jerk the bot
 */
static void finish_turn(int brake_dir)
{
    pivot(brake_dir, BRAKE_DUTY);
    vTaskDelay(BRAKE_MS / portTICK_PERIOD_MS);
    stop();
    pid_reset();
}

void back_to_follow(maze_ctx_t *ctx)
{
    pid_reset();
    ctx->last_centred = true;
    ctx->junction_armed = false;
    set_state(ctx, STATE_FOLLOW);
}

void start_pivot(maze_ctx_t *ctx, int dir)
{
    ctx->turn_dir = dir;
    // If the straight line is still under the bar, the pivot has to leave it first
    ctx->left_old_line = !ctx->lsa.on_line[2];
    set_state(ctx, STATE_PIVOT);
    pivot(dir, TURN_DUTY);
}

void start_sharp(maze_ctx_t *ctx, int dir)
{
    ctx->turn_dir = dir;
    set_state(ctx, STATE_SHARP);
    if (dir < 0)
        drive(-SHARP_INNER_DUTY, SHARP_OUTER_DUTY);
    else
        drive(SHARP_OUTER_DUTY, -SHARP_INNER_DUTY);
}

void start_uturn(maze_ctx_t *ctx)
{
    set_state(ctx, STATE_UTURN);
    pivot(UTURN_DIR, TURN_DUTY);
}

void start_search(maze_ctx_t *ctx, int dir)
{
    ctx->turn_dir = dir;
    ctx->search_timeout = SEARCH_TIMEOUT_MS;
    set_state(ctx, STATE_SEARCH);
    pivot(dir, SEARCH_DUTY);
}

/*
 * PIVOT: first the centre sensor must leave the line we were on, then the turn
 * ends as soon as it lands on the next line (so this also lands on 45 degree exits)
 */
void handle_pivot(maze_ctx_t *ctx)
{
    if (!ctx->lsa.on_line[2])
    {
        ctx->left_old_line = true;
    }

    if (ctx->left_old_line && state_elapsed(ctx) >= PIVOT_MIN_MS && ctx->lsa.on_line[2])
    {
        finish_turn(-ctx->turn_dir);
        back_to_follow(ctx);
        return;
    }

    if (state_elapsed(ctx) >= PIVOT_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "pivot timed out");
        start_lost(ctx, ctx->turn_dir);
    }
}

/*
 * SHARP: keep swinging until the centre sensor is on the line
 */
void handle_sharp(maze_ctx_t *ctx)
{
    if (ctx->lsa.on_line[2])
    {
        back_to_follow(ctx);
        return;
    }

    if (state_elapsed(ctx) >= SHARP_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "sharp turn timed out");
        start_lost(ctx, ctx->turn_dir);
    }
}

/*
 * UTURN: spin until the centre finds the line we came in on. after_uturn makes
 * the next node skip straight, which would lead back toward where we turned.
 */
void handle_uturn(maze_ctx_t *ctx)
{
    if (state_elapsed(ctx) >= UTURN_MIN_MS && ctx->lsa.on_line[2])
    {
        finish_turn(-UTURN_DIR);
        ctx->after_uturn = true;
        back_to_follow(ctx);
        return;
    }

    if (state_elapsed(ctx) >= UTURN_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "U-turn timed out");
        start_lost(ctx, UTURN_DIR);
    }
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
        back_to_follow(ctx);
        return;
    }

    if (state_elapsed(ctx) >= ctx->search_timeout)
    {
        // Sweep back the other way, twice as far so we cover the first side again
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
    display_update(ctx);

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
        ctx->seen_line = false;
        ctx->hold_until_ms = now_ms() + START_HOLD_MS;
        back_to_follow(ctx);
    }
}
