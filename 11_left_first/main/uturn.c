/*
 * Dedicated dead-end / obstacle U-turn state.
 * The wheels counter-rotate, so the robot turns around its centre instead of
 * following a wide-radius arc. Ignore the line until roughly 180 degrees of
 * pivot time has elapsed, then reacquire it with the centre sensor.
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"

static const char *TAG = "uturn";

void start_uturn(maze_ctx_t *ctx)
{
    ESP_LOGI(TAG, "starting in-place 180 degree pivot (%s)", UTURN_DIR < 0 ? "left" : "right");
    ctx->turn_dir = UTURN_DIR;
    ctx->left_old_line = !ctx->lsa.on_line[2];
    ctx->turn_slowed = false;
    ctx->after_uturn = false;
    set_state(ctx, STATE_UTURN);
    pivot(ctx->turn_dir, TURN_DUTY);
}

void handle_uturn(maze_ctx_t *ctx)
{
    uint32_t elapsed = state_elapsed(ctx);
    if (!ctx->lsa.on_line[2])
        ctx->left_old_line = true;

    // A calibrated 90 degree pivot gives a better 180 degree minimum. Before the
    // first calibration, use the configured safe minimum instead.
    uint32_t half_turn_ms = ctx->pivot_ms ? ctx->pivot_ms * 2 : UTURN_MIN_MS;

    // Ease off before the centre sensor reaches the return line. The adjacent
    // sensor on the turn side sees it first; slowing here reduces angular coast.
    int near_sensor = ctx->turn_dir < 0 ? 1 : 3;
    uint32_t slow_start_ms = half_turn_ms * 3 / 4;
    if (slow_start_ms < UTURN_MIN_MS)
        slow_start_ms = UTURN_MIN_MS;
    if (!ctx->turn_slowed && elapsed >= slow_start_ms && ctx->lsa.on_line[near_sensor])
    {
        ctx->turn_slowed = true;
        pivot(ctx->turn_dir, TURN_SLOW_DUTY);
    }

    if (ctx->left_old_line && elapsed >= half_turn_ms && ctx->lsa.on_line[2])
    {
        ctx->after_uturn = true;
        finish_turn(ctx);
        return;
    }

    if (elapsed >= UTURN_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "U-turn line reacquisition timed out; entering SEARCH");
        ctx->after_uturn = true;
        start_search(ctx, ctx->turn_dir);
    }
}
