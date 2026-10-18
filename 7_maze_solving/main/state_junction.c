/*
 * APPROACH state: a side branch was seen in FOLLOW.
 *
 * The sensor bar is ahead of the wheels, so drive straight for APPROACH_MS until
 * the wheels are on the junction, then look at what is under the bar:
 *   - whole bar still white         -> end zone, DONE
 *   - one exit                      -> plain corner, not counted
 *   - two or more exits             -> node, counted, exit picked by choose_exit()
 * then go straight (FOLLOW), turn (PIVOT) or, if nothing is there, SEARCH.
 */
#include "sra_board.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "pid.h"
#include "junction.h"
#include "drop_zone.h"

static const char *TAG = "maze";

void handle_approach(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;

    // The pink square can look like a wide patch too, it wins over the end zone
    if (drop_zone_check(ctx) == DROP_CONFIRMED)
    {
        start_drop(ctx);
        return;
    }

    // A cross can show 11100 first and 11111 a moment later, so keep collecting
    ctx->seen_branches |= detect_branches(lsa);

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
        stop();
        ESP_LOGI(TAG, "END zone reached, %d nodes", ctx->node_count);
        set_state(ctx, STATE_DONE);
        return;
    }

    // Exits available here. Straight = line still under the middle of the bar.
    bool left = ctx->seen_branches & BRANCH_LEFT;
    bool right = ctx->seen_branches & BRANCH_RIGHT;
    bool straight = lsa->on_line[1] || lsa->on_line[2] || lsa->on_line[3];
    // One exit = just a corner, two or more = a node (counted)
    bool is_node = left + right + straight >= 2;

    char plan = 'A';
    if (is_node)
    {
        ctx->node_count++;
        ctx->display_dirty = true;
        plan = node_plan_for(ctx->node_count);
    }

    exit_t choice = choose_exit(left, straight, right, ctx->after_uturn, plan);
    ESP_LOGI(TAG, "%s %d  exits L%d S%d R%d  plan %c  after_uturn %d -> %s",
             is_node ? "node" : "corner", is_node ? ctx->node_count : 0,
             left, straight, right, plan, ctx->after_uturn, exit_name(choice));
    if (is_node)
    {
        ctx->after_uturn = false;
    }

    // Act on the choice: carry on, turn, or search if nothing was found
    if (choice == EXIT_STRAIGHT)
    {
        back_to_follow(ctx);
    }
    else if (choice == EXIT_NONE)
    {
        start_search(ctx, TURN_PREFERENCE);
    }
    else
    {
        start_pivot(ctx, choice == EXIT_LEFT ? -1 : 1);
    }
}
