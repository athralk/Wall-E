/*
 * Backtracking, see backtrack.h
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "backtrack.h"

static const char *TAG = "backtrack";

/*
 * Ring buffer of wheel duties, one entry per loop. When full the oldest entry
 * is overwritten, so it holds the last BACKTRACK_MAX_MS of driving.
 */
#define HISTORY_LEN (BACKTRACK_MAX_MS / LOOP_MS)

static int8_t history_left[HISTORY_LEN], history_right[HISTORY_LEN];
static int head = 0;    // next slot to write
static int size = 0;    // entries stored

void backtrack_record(void)
{
    float left, right;
    motion_last(&left, &right);
    history_left[head] = (int8_t)left;
    history_right[head] = (int8_t)right;
    head = (head + 1) % HISTORY_LEN;
    if (size < HISTORY_LEN)
    {
        size++;
    }
}

void backtrack_mark_good(void)
{
    size = 0;
}

/*
 * Takes the newest entry off the history, false if it is empty
 */
static bool pop(float *left, float *right)
{
    if (size == 0)
    {
        return false;
    }
    head = (head + HISTORY_LEN - 1) % HISTORY_LEN;
    size--;
    *left = history_left[head];
    *right = history_right[head];
    return true;
}

void start_lost(maze_ctx_t *ctx, int dir)
{
    ctx->turn_dir = dir;   // used by SEARCH if backtracking doesn't work

    if (now_ms() - ctx->last_backtrack_ms > BACKTRACK_RESET_MS)
    {
        ctx->backtrack_count = 0;
    }

    if (size > 0 && ctx->backtrack_count < BACKTRACK_TRIES)
    {
        ctx->backtrack_count++;
        ctx->last_backtrack_ms = now_ms();
        ESP_LOGW(TAG, "confused, backing up %d ms (try %d of %d)", size * LOOP_MS,
                 ctx->backtrack_count, BACKTRACK_TRIES);
        set_state(ctx, STATE_BACKTRACK);
        return;
    }

    start_search(ctx, dir);
}

void handle_backtrack(maze_ctx_t *ctx)
{
    // Back on the line: carry on from here
    if (ctx->lsa.on_line[2])
    {
        stop();
        size = 0;
        back_to_follow(ctx);
        return;
    }

    // Replay the path backwards: newest command first, both wheels reversed
    float left, right;
    if (!pop(&left, &right))
    {
        ESP_LOGW(TAG, "back at the start of the history, no line, searching");
        start_search(ctx, ctx->turn_dir);
        return;
    }
    drive(-left, -right);
}
