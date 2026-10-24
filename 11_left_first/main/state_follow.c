/*
 * FOLLOW state: normal PID line following.
 *
 * Every loop it checks, in this order, and hands off to another state if one fires:
 *   1. side branch (junction)  -> APPROACH (state_junction.c), then a 90 degree turn
 *   2. line gone               -> TURN_120 toward a separate side line seen just before
 *                                 (acute exit), U-turn if it was centred (dead end),
 *                                 else SEARCH toward where it was
 *   3. none of the above: PID line following. Curves and 45 degree bends are just followed
 *      (the PID steers round them), no turn state needed
 * (Pink drop zone check is commented out.)
 *
 * Checks 1-3 are skipped (bot drives straight if it sees no line) for START_HOLD_MS
 * at power on and INVERT_HOLD_MS after a colour flip, so the start and the edges of
 * the colour blind block can't look like a junction or a dead end.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "pid.h"
#include "junction.h"
// #include "drop_zone.h"     // pink zone off
#include "turn_90.h"
#include "turn_120.h"
#include "display.h"

static const char *TAG = "maze";

void handle_follow(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;

    // Pink drop zone: off for now
    // drop_check_t pink = drop_zone_check(ctx);
    // if (pink == DROP_CONFIRMED)
    // {
    //     start_drop(ctx);
    //     return;
    // }
    // if (pink == DROP_MAYBE)
    // {
    //     drive(BASE_DUTY, BASE_DUTY);
    //     return;
    // }

    // Start: motors off until the bar has been on a line for START_ON_LINE_MS, so the bot
    // doesn't drive / spin while it is in your hand or being put down
    if (!ctx->seen_line)
    {
        if (lsa->count == 0)
        {
            ctx->line_seen_since_ms = 0;
            stop();
            return;
        }
        if (ctx->line_seen_since_ms == 0)
        {
            ctx->line_seen_since_ms = now_ms();
        }
        if (now_ms() - ctx->line_seen_since_ms < START_ON_LINE_MS)
        {
            stop();
            return;
        }
        ESP_LOGI(TAG, "on the line, starting");
        ctx->line_seen_since_ms = 0;
        ctx->seen_line = true;
        ctx->hold_until_ms = 0;
        ctx->leaving_start = true;
        pid_reset();
        // Break free from standing still: both wheels hard forward for a moment (at the
        // follow speeds one wheel could sit still until the bot was nudged)
        drive(START_PUSH_DUTY, START_PUSH_DUTY);
        vTaskDelay(START_PUSH_MS / portTICK_PERIOD_MS);
        ctx->kick_until_ms = now_ms() + KICK_MS;
    }

    // Started on a white patch (start zone): all five sensors white is not a junction or
    // the end zone here, just drive straight out of it until the bar sees a normal line
    if (ctx->leaving_start)
    {
        if (lsa->count == LSA_COUNT)
        {
            drive(KICK_DUTY, KICK_DUTY);
            return;
        }
        ctx->leaving_start = false;
    }
    bool hold = now_ms() < ctx->hold_until_ms;

    if (!hold)
    {
        // After a junction / turn, wait for REARM_LOOPS plain line readings in a row before
        // looking for the next turn. Stops the bot from re-triggering on the corner it just
        // turned at (the cause of turning back and forth at the same spot).
        int branches = detect_branches(lsa, line_width_typical(&ctx->line_width), ctx->last_centred);
        int side_line = detect_side_line(lsa);
        line_width_update(&ctx->line_width, lsa);
        if (ctx->rearm_count > 0)
        {
            // The front object detector is separate from LSA branch re-arming.
            bool plain = branches == 0 && lsa->count > 0;
            ctx->rearm_count = plain ? ctx->rearm_count - 1 : REARM_LOOPS;
            branches = 0;
            side_line = 0;  // the line we just turned off can still be beside the new one
        }
        // A separate second line beside the main one: remember when, for the acute exit check
        // (or a confirmed-width branch on that side: the arm can merge with the main line)
        if ((side_line & BRANCH_LEFT) || (branches & BRANCH_LEFT))
            ctx->side_left_ms = now_ms();
        if ((side_line & BRANCH_RIGHT) || (branches & BRANCH_RIGHT))
            ctx->side_right_ms = now_ms();

        // 1. Side branch held for CORNER_CONFIRM loops in a row: drive onto the junction.
        //    hits_left / hits_right count the loops each side was seen; APPROACH only takes a
        //    side as an exit if it was seen in CORNER_CONFIRM loops, so one stray reading of
        //    the other side can't add an exit.
        if (branches)
        {
            ctx->node_confirm++;
            if (branches & BRANCH_LEFT)
                ctx->hits_left++;
            if (branches & BRANCH_RIGHT)
                ctx->hits_right++;
        }
        else
        {
            ctx->node_confirm = ctx->hits_left = ctx->hits_right = 0;
        }
        if (ctx->node_confirm >= CORNER_CONFIRM)
        {
            ctx->node_confirm = 0;
            set_state(ctx, STATE_APPROACH);
            drive(APPROACH_DUTY, APPROACH_DUTY);
            return;
        }
        if (ctx->node_confirm > 0)
        {
            // Maybe a branch: don't let the PID swing toward it while it is being confirmed,
            // that swing is what used to make the bot lose its line at junctions
            drive(BASE_DUTY, BASE_DUTY);
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

        if (line_sensor_is_inverted() && lsa->raw_white_count == LSA_COUNT && ctx->last_centred)
        {
            // Black line mode, black line ended while centred, bar all white: crossing the
            // end of the block. Keep going straight until colour_blind.c sees 01110.
            drive(BASE_DUTY, BASE_DUTY);
            return;
        }

        ctx->lost_count++;
        if (ctx->lost_since_ms == 0)
            ctx->lost_since_ms = now_ms();

        // Line gone shortly after a separate second line was seen on one side (10110...): that
        // was the arm of an acute exit. Pivot that way (left first if both), the first line the
        // centre finds is the exit. Without this the PID, pulled toward the arm, leaves the main
        // line on the other side and the search went the wrong way.
        uint32_t t = now_ms();
        bool recent_left = ctx->side_left_ms && t - ctx->side_left_ms < ACUTE_WINDOW_MS;
        bool recent_right = ctx->side_right_ms && t - ctx->side_right_ms < ACUTE_WINDOW_MS;
        if ((recent_left || recent_right) && ctx->lost_count >= LOST_CONFIRM)
        {
            // Both sides seen: the one seen last is where the arm is
            int dir = !recent_right ? -1 : !recent_left ? 1
                      : (ctx->side_left_ms >= ctx->side_right_ms ? -1 : 1);
            ESP_LOGI(TAG, "line ended %lu ms after a line was seen on the %s: acute %s",
                     (unsigned long)(t - (dir < 0 ? ctx->side_left_ms : ctx->side_right_ms)),
                     dir < 0 ? "left" : "right", dir < 0 ? "left" : "right");
            ctx->lost_count = 0;
            ctx->side_left_ms = ctx->side_right_ms = 0;
            start_turn_120(ctx, dir);
            return;
        }

        if (ctx->last_centred && ctx->centred_count >= CENTRED_CONFIRM)
        {
            // Line ended while it had sat in the middle of the bar for a while: dead end, but
            // only once it has stayed gone DEADEND_MS of driving straight, so a gap or a thin
            // line slipping between two sensors isn't taken as one
            drive(BASE_DUTY, BASE_DUTY);
            if (now_ms() - ctx->lost_since_ms >= DEADEND_MS)
            {
                ctx->lost_count = 0;
                ctx->lost_since_ms = 0;
                ESP_LOGI(TAG, "line lost while centred; searching before committing to a U-turn");
                start_search(ctx, TURN_PREFERENCE);
            }
        }
        else if (ctx->lost_count >= LOST_CONFIRM)
        {
            // Line slipped off one side, or only just reached the middle (a missed turn)
            ctx->lost_count = 0;
            ctx->lost_since_ms = 0;
            // A corner the junction check missed (00111 -> 00000): pivot toward the side the
            // sensors last saw the line on. (Not the PID error: it is 0 after every turn and
            // stale while a branch is being checked, which turned the bot the wrong way.)
            int dir = ctx->line_side ? ctx->line_side : -1;
            ESP_LOGI(TAG, "line lost, last seen on the %s", ctx->line_side > 0 ? "right"
                     : ctx->line_side < 0 ? "left" : "middle (turning left)");
            start_turn_90(ctx, dir);
        }
        // else brief gap: keep the last motor output going
        return;
    }
    ctx->lost_count = 0;
    ctx->lost_since_ms = 0;
    // Remember if the line is under the middle of the bar (centre sensor on, outer sensors off:
    // 00100, 01110, 01100, 00110), used for the dead end check above. Only a line that has
    // slid out to an outer sensor is the start of a missed turn, so losing it then means search
    // toward it. Before, a dead end where the line ended slightly off centre (01100) was taken
    // for a missed turn and the bot swept left and right instead of U-turning.
    ctx->last_centred = lsa->on_line[2] && !lsa->on_line[0] && !lsa->on_line[LSA_COUNT - 1];
    ctx->centred_count = ctx->last_centred ? ctx->centred_count + 1 : 0;
    // Which side the line is on, from the sensors: kept until it is on the other side
    if (lsa->position <= -0.5f || ((lsa->on_line[0] || lsa->on_line[1]) && !lsa->on_line[3] && !lsa->on_line[4]))
        ctx->line_side = -1;
    else if (lsa->position >= 0.5f || ((lsa->on_line[3] || lsa->on_line[4]) && !lsa->on_line[0] && !lsa->on_line[1]))
        ctx->line_side = 1;
    // 4. Normal PID following: speed up one wheel and slow the other. Just after a stop
    //    (turn, start) use the kick speeds so neither wheel sits below what gets it moving.
    bool kick = now_ms() < ctx->kick_until_ms;
    int base = kick ? KICK_DUTY : BASE_DUTY;
    int low = kick ? KICK_MIN_DUTY : MIN_DUTY;
    float correction = pid_compute(lsa->position);
    // Line out at the edge of the bar (curve, 45 degree bend): steer harder toward it
    if (lsa->position >= EDGE_BOOST_POS)
        correction += EDGE_BOOST_DUTY;
    else if (lsa->position <= -EDGE_BOOST_POS)
        correction -= EDGE_BOOST_DUTY;
    float left = base + correction, right = base - correction;
    // Inner wheel would drop below the minimum (where it stalls): lift both wheels by the
    // shortfall, so the bot still steers by the same difference but no wheel stops
    float lift = low - (left < right ? left : right);
    if (lift > 0)
    {
        left += lift;
        right += lift;
    }
    drive(bound(left, low, MAX_DUTY), bound(right, low, MAX_DUTY));
}
