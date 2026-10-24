/*
 * Acute (~120 degree) turns: a direct pivot, like the 90 degree turn.
 *
 * Coming up to an acute exit, its arm runs back under the bar on its side (a separate
 * second line, e.g. 10110, or a branch) just before the main line ends. state_follow.c
 * spots that and starts this turn toward the arm. The bar has no line under it when the
 * turn starts, and the first line the centre sensor meets turning that way is the arm (the
 * line the bot came in on is further round). Pivot at a steady TURN_DUTY until the centre
 * sensor is on it, then finish_turn().
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "turn_120.h"

static const char *TAG = "turn_120";

void start_turn_120(maze_ctx_t *ctx, int dir)
{
    ESP_LOGI(TAG, "120 %s", dir < 0 ? "left" : "right");
    ctx->turn_dir = dir;
    set_state(ctx, STATE_TURN_120);
    pivot(dir, TURN_DUTY);
}

void handle_turn_120(maze_ctx_t *ctx)
{
    if (ctx->lsa.on_line[2])
    {
        ctx->after_uturn = false;
        finish_turn(ctx);
        return;
    }

    if (state_elapsed(ctx) >= ACUTE_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "no line found, searching");
        start_search(ctx, ctx->turn_dir);
    }
}
