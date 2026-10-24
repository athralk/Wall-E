/*
 * APPROACH state: a side branch was seen in FOLLOW.
 *
 * The sensor bar is ahead of the wheels, so drive straight for APPROACH_MS until
 * the wheels are on the junction, then look at what is under the bar:
 *   - whole bar still white         -> end zone, DONE
 *   - one exit                      -> plain corner, not counted
 *   - two or more exits             -> node, counted, exit picked by choose_exit()
 *                                      (or by node_memory.c when coming back from a dead end)
 * then go straight (FOLLOW), turn (PIVOT) or, if nothing is there, SEARCH.
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "pid.h"
#include "junction.h"
// #include "drop_zone.h"     // pink zone off
#include "turn_90.h"
#include "node_memory.h"

static const char *TAG = "maze";

void handle_approach(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;

    // The pink square can look like a wide patch too, it wins over the end zone
    // Pink drop zone: off for now
    // if (drop_zone_check(ctx) == DROP_CONFIRMED)
    // {
    //     start_drop(ctx);
    //     return;
    // }

    // A cross can show 11110 first and 11111 a moment later, so keep counting each side.
    // The bot drives straight here (no PID), so the line under the bar doesn't wobble.
    int branches = detect_branches(lsa, line_width_typical(&ctx->line_width), false);
    if (branches & BRANCH_LEFT)
        ctx->hits_left++;
    if (branches & BRANCH_RIGHT)
        ctx->hits_right++;
    if (lsa->ir_left)
    {
        // Side IR over a line left of the main one: a left exit (90 degree or acute)
        ctx->hits_left = CORNER_CONFIRM;
        ctx->ir_seen = true;
        ctx->side_left_ms = now_ms();
    }

    if (state_elapsed(ctx) < APPROACH_MS)
    {
        return;
    }

    // Still white under the whole bar after driving onto it: maybe the end zone.
    // Keep driving for END_CONFIRM_MS and only stop if it is still all white, so a wide
    // junction or a bit of glare can't stop the run. Never in the colour blind section.
    bool all_white = lsa->raw_white_count == LSA_COUNT && !line_sensor_is_inverted();
    if (all_white && state_elapsed(ctx) < APPROACH_MS + END_CONFIRM_MS)
    {
        return;
    }
    if (all_white)
    {
        ctx->line_seen_since_ms = 0;
        stop();
        ESP_LOGI(TAG, "END zone reached, %d nodes", ctx->node_count);
        set_state(ctx, STATE_DONE);
        return;
    }

    // Exits available here: a side only if it was seen in CORNER_CONFIRM loops (FOLLOW +
    // APPROACH), so one stray reading can't add it. Straight = line still under the middle.
    bool left = ctx->hits_left >= CORNER_CONFIRM;
    bool right = ctx->hits_right >= CORNER_CONFIRM;
    bool straight = lsa->on_line[1] || lsa->on_line[2] || lsa->on_line[3];
    if (!left && !straight && !right && state_elapsed(ctx) < APPROACH_MS + APPROACH_RECHECK_MS)
    {
        // A single empty reading at a node can send the robot into a rotating search.
        // Keep moving forward briefly so the next LSA samples can reveal the exits.
        drive(APPROACH_DUTY, APPROACH_DUTY);
        return;
    }

    // One exit = just a corner, two or more = a node (counted)
    bool is_node = left + right + straight >= 2;

    // Under a wall-following rule, a U-turn returns along the same route and the
    // first junction reached is normally one already counted. Do not count that
    // return pass as a new node. Straight-first node-memory keeps its visit-based
    // behavior because it uses the count to track the return stack.
    bool returning_to_seen_node = ctx->after_uturn && EXIT_RULE != RULE_STRAIGHT_FIRST;
    bool count_node = is_node && !returning_to_seen_node;

    char plan = 'A';
    if (count_node)
    {
        ctx->node_count++;
        ctx->display_dirty = true;
        plan = node_plan_for(ctx->node_count);
    }

    exit_t choice = EXIT_NONE;
    // Node memory is only needed for straight first: wall followers handle dead ends themselves
    bool returning = ctx->after_uturn && EXIT_RULE == RULE_STRAIGHT_FIRST && NODE_MEMORY;
    if (is_node && returning)
    {
        // Back at a node after a dead end: take the next exit we haven't tried here.
        // If they were all dead ends, keep returning toward the node before.
        bool still_returning;
        choice = node_memory_return(left, straight, right, &still_returning);
        if (choice == EXIT_NONE)
        {
            // Nothing remembered (or the remembered exit isn't there): old rule
            choice = choose_exit(left, straight, right, true, plan);
            still_returning = false;
        }
        ctx->after_uturn = still_returning;
    }
    else
    {
        choice = choose_exit(left, straight, right, ctx->after_uturn, plan);
        if (is_node)
        {
            // New node: remember it in case the exit we take is a dead end
            node_memory_push(left, straight, right, choice);
            ctx->after_uturn = false;
        }
    }
    ESP_LOGI(TAG, "%s %d  exits L%d S%d R%d  (hits L%d R%d, line width %d)  plan %c  returning %d -> %s",
             is_node ? "node" : "corner", is_node ? ctx->node_count : 0, left, straight, right,
             ctx->hits_left, ctx->hits_right, line_width_typical(&ctx->line_width), plan, returning,
             exit_name(choice));
    ctx->ir_seen = false;
    ctx->hits_left = ctx->hits_right = 0;

    // Act on the choice: carry on, turn, or search if nothing was found
    if (choice == EXIT_STRAIGHT)
    {
        back_to_follow(ctx);
    }
    else if (choice == EXIT_NONE)
    {
        ESP_LOGW(TAG, "junction approach found no exit after resampling; searching");
        start_search(ctx, TURN_PREFERENCE);
    }
    else
    {
        start_turn_90(ctx, choice == EXIT_LEFT ? -1 : 1);
    }
}
