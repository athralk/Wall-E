/*
 * The turning states. Each start_*() sets the state and the motors, each
 * handle_*() runs every loop and goes back to FOLLOW once the centre sensor
 * is on a line. Any turn that takes too long falls back to SEARCH.
 *
 *   TURN    every turn in place (90 degree, 45 degree / acute, U-turn), see turn_kind_t
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
    // Turns end stopped, and BASE_DUTY can't always get the wheels going from standstill
    ctx->kick_until_ms = now_ms() + KICK_MS;
    ctx->turn_end_ms = now_ms();
    ctx->last_centred = true;
    ctx->centred_count = 0;
    ctx->lost_count = 0;
    ctx->lost_since_ms = 0;
    ctx->rearm_count = REARM_LOOPS;
    set_state(ctx, STATE_FOLLOW);
}

static const char *turn_name(turn_kind_t kind)
{
    switch (kind)
    {
    case TURN_PIVOT: return "pivot";
    case TURN_SHARP: return "sharp turn";
    case TURN_UTURN: return "U-turn";
    }
    return "?";
}

void start_turn(maze_ctx_t *ctx, turn_kind_t kind, int dir)
{
    ESP_LOGI(TAG, "%s %s", turn_name(kind), dir < 0 ? "left" : "right");
    ctx->turn_kind = kind;
    ctx->turn_dir = dir;
    // If the centre sensor is still on the line we are turning away from, it has to come
    // off it first, so the turn can't end on the line it started on
    ctx->left_old_line = !ctx->lsa.on_line[2];
    ctx->turn_slowed = false;
    ctx->turn_near = false;
    set_state(ctx, STATE_TURN);
    // Start hard to get the wheels turning, handle_turn() drops to TURN_DUTY
    pivot(dir, TURN_KICK_DUTY);
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
 * TURN: one handler for every turn. Pivot until the centre sensor has come off the old
 * line and then lands on a line, ignoring the line for the first min_ms. The turn kinds
 * only differ in min_ms / timeout and what gets recorded at the end.
 */
void handle_turn(maze_ctx_t *ctx)
{
    static const uint32_t min_ms[] = {
        [TURN_PIVOT] = PIVOT_MIN_MS,
        [TURN_SHARP] = 0,
        [TURN_UTURN] = UTURN_MIN_MS,  // so a U-turn doesn't grab the stub it just left
    };
    static const uint32_t timeout_ms[] = {
        [TURN_PIVOT] = PIVOT_TIMEOUT_MS,
        [TURN_SHARP] = SHARP_TIMEOUT_MS,
        [TURN_UTURN] = UTURN_TIMEOUT_MS,
    };
    turn_kind_t kind = ctx->turn_kind;
    uint32_t elapsed = state_elapsed(ctx);

    if (!ctx->turn_slowed && elapsed >= TURN_KICK_MS)
    {
        ctx->turn_slowed = true;
        if (!ctx->turn_near)
        {
            pivot(ctx->turn_dir, TURN_DUTY);
        }
    }

    // New line reaching the sensors on the turn side (they lead the centre one): slow down
    // so the centre eases onto it and the brake has less to stop
    const bool *on = ctx->lsa.on_line;
    bool side_sees = ctx->turn_dir < 0 ? (on[0] || on[1]) : (on[3] || on[4]);
    if (!ctx->turn_near && ctx->left_old_line && elapsed >= min_ms[kind] && side_sees)
    {
        ctx->turn_near = true;
        ctx->turn_slowed = true;
        pivot(ctx->turn_dir, TURN_SLOW_DUTY);
    }

    if (!ctx->lsa.on_line[2])
    {
        ctx->left_old_line = true;
    }

    // Landed: the centre sensor is on the new line. For 90 degree turns and U-turns also the
    // sensor just before it on the turn side (A1 turning left, A3 turning right): stopping
    // there, half a sensor early, cancels the overshoot from the bot's momentum, and FOLLOW
    // centres the rest. Not for the 45 degree turn, its line starts next to that sensor.
    bool inner = ctx->turn_dir < 0 ? on[1] : on[3];
    bool landed = on[2] || (kind != TURN_SHARP && ctx->turn_near && inner);
    if (ctx->left_old_line && elapsed >= min_ms[kind] && landed)
    {
        if (kind == TURN_PIVOT)
        {
            ctx->pivot_ms = elapsed;    // 90 degree pivot time, used by search_turned_around()
        }
        if (kind == TURN_UTURN)
        {
            ctx->after_uturn = true;
        }
        finish_turn(-ctx->turn_dir);
        back_to_follow(ctx);
        return;
    }

    if (elapsed >= timeout_ms[kind])
    {
        ESP_LOGW(TAG, "%s timed out", turn_name(kind));
        if (kind == TURN_UTURN)
        {
            ctx->after_uturn = true;    // we were turning round at a dead end either way
        }
        start_search(ctx, ctx->turn_dir);
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
