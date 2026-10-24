/*
 * 90 degree turns and the U-turn: a direct pivot in place, the way it worked before.
 *
 *   1. pivot at a steady TURN_DUTY
 *   2. the centre sensor must first come off the line the turn started on (left_old_line),
 *      so the turn can't end on that line, and PIVOT_MIN_MS must have passed
 *   3. stop turning as soon as the centre sensor is on the new line: finish_turn() gives a
 *      short reverse pulse against the spin (kills the overshoot) and hands straight back
 *      to line following with the wheels driving forward
 *
 * A U-turn ignores the line for UTURN_MIN_MS, so it doesn't grab the stub it just left.
 * No line in time: SEARCH.
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "turn_90.h"

static const char *TAG = "turn_90";

static void begin(maze_ctx_t *ctx, turn_kind_t kind, int dir)
{
    ESP_LOGI(TAG, "%s %s", kind == TURN_UTURN ? "U-turn" : "90", dir < 0 ? "left" : "right");
    ctx->turn_kind = kind;
    ctx->turn_dir = dir;
    ctx->left_old_line = !ctx->lsa.on_line[2];
    set_state(ctx, STATE_TURN_90);
    pivot(dir, TURN_DUTY);
}

void start_turn_90(maze_ctx_t *ctx, int dir)
{
    begin(ctx, TURN_PIVOT, dir);
}

void start_uturn(maze_ctx_t *ctx)
{
    begin(ctx, TURN_UTURN, UTURN_DIR);
}

void handle_turn_90(maze_ctx_t *ctx)
{
    uint32_t elapsed = state_elapsed(ctx);
    bool uturn = ctx->turn_kind == TURN_UTURN;

    if (!ctx->lsa.on_line[2])
        ctx->left_old_line = true;

    if (ctx->left_old_line && elapsed >= (uturn ? UTURN_MIN_MS : PIVOT_MIN_MS) && ctx->lsa.on_line[2])
    {
        if (!uturn)
            ctx->pivot_ms = elapsed;    // 90 degree pivot time, used by SEARCH
        ctx->after_uturn = uturn;
        finish_turn(ctx);
        return;
    }

    if (elapsed >= (uturn ? UTURN_TIMEOUT_MS : PIVOT_TIMEOUT_MS))
    {
        ESP_LOGW(TAG, "no line found, searching");
        ctx->after_uturn = uturn;
        start_search(ctx, ctx->turn_dir);
    }
}
