/* 90 degree turns pivot in place until the centre sensor finds the new line. */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "turn_90.h"

static const char *TAG = "turn_90";

void start_turn_90(maze_ctx_t *ctx, int dir)
{
    ESP_LOGI(TAG, "90 %s", dir < 0 ? "left" : "right");
    ctx->turn_dir = dir;
    ctx->left_old_line = !ctx->lsa.on_line[2];
    set_state(ctx, STATE_TURN_90);
    pivot(dir, TURN_DUTY);
}

void handle_turn_90(maze_ctx_t *ctx)
{
    uint32_t elapsed = state_elapsed(ctx);
    if (!ctx->lsa.on_line[2])
        ctx->left_old_line = true;

    if (ctx->left_old_line && elapsed >= PIVOT_MIN_MS && ctx->lsa.on_line[2])
    {
        ctx->pivot_ms = elapsed;    // 90 degree pivot time, used by SEARCH and UTURN
        ctx->after_uturn = false;
        finish_turn(ctx);
        return;
    }

    if (elapsed >= PIVOT_TIMEOUT_MS)
    {
        ESP_LOGW(TAG, "no line found, searching");
        start_search(ctx, ctx->turn_dir);
    }
}
