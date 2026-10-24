/*
 * Junction / turn pattern checks and the exit choice, see junction.h
 */
#include <string.h>
#include "config.h"
#include "junction.h"

/*
 * Lit sensors as blocks: a block is a run of neighbouring sensors that are all on.
 * Fills first / last of each block (up to LSA_COUNT blocks), returns how many.
 */
static int find_blocks(const lsa_state_t *lsa, int first[], int last[])
{
    int blocks = 0;
    for (int i = 0; i < LSA_COUNT; i++)
    {
        if (!lsa->on_line[i])
            continue;
        if (i == 0 || !lsa->on_line[i - 1])
        {
            first[blocks] = i;
            blocks++;
        }
        last[blocks - 1] = i;
    }
    return blocks;
}

void line_width_reset(line_width_t *lw)
{
    lw->count = 0;
    lw->next = 0;
}

void line_width_update(line_width_t *lw, const lsa_state_t *lsa)
{
    int first[LSA_COUNT], last[LSA_COUNT];
    // Plain = one block that doesn't touch an outer sensor: only the main line is under the bar
    if (find_blocks(lsa, first, last) != 1 || first[0] == 0 || last[0] == LSA_COUNT - 1)
        return;
    lw->width[lw->next] = last[0] - first[0] + 1;
    lw->next = (lw->next + 1) % LINE_WIDTH_HISTORY;
    if (lw->count < LINE_WIDTH_HISTORY)
        lw->count++;
}

int line_width_typical(const line_width_t *lw)
{
    // Not enough seen yet (start, just after a colour flip): assume the widest a plain
    // reading can be, so nothing can look wider than the line by mistake
    if (lw->count < LINE_WIDTH_MIN_SAMPLES)
        return LSA_COUNT - 2;
    int widest = 0;
    for (int i = 0; i < lw->count; i++)
    {
        if (lw->width[i] > widest)
            widest = lw->width[i];
    }
    return widest;
}

int detect_branches(const lsa_state_t *lsa, int typical_width, bool was_centred)
{
    int first[LSA_COUNT], last[LSA_COUNT];
    if (find_blocks(lsa, first, last) != 1)
        return 0;

    int width = last[0] - first[0] + 1;
    const int centre = LSA_COUNT / 2;
    // Require the branch to be wider than the learned normal line even when the
    // previous sample was centred. A smooth curve can move a normal-width line onto
    // an outer sensor; treating that as a branch sends the robot into APPROACH.
    (void)was_centred;
    int need = typical_width + 1;
    if (width < need || first[0] > centre || last[0] < centre)
        return 0;

    int branches = 0;
    if (first[0] == 0)
        branches |= BRANCH_LEFT;
    if (last[0] == LSA_COUNT - 1)
        branches |= BRANCH_RIGHT;
    return branches;
}

int detect_side_line(const lsa_state_t *lsa)
{
    int first[LSA_COUNT], last[LSA_COUNT];
    int blocks = find_blocks(lsa, first, last);
    if (blocks < 2)
        return 0;

    int sides = 0;
    if (first[0] == 0)
        sides |= BRANCH_LEFT;
    if (last[blocks - 1] == LSA_COUNT - 1)
        sides |= BRANCH_RIGHT;
    return sides;
}

int detect_sharp(const lsa_state_t *lsa)
{
    bool centre_and_right = lsa->on_line[2] || lsa->on_line[3] || lsa->on_line[4];
    bool centre_and_left = lsa->on_line[0] || lsa->on_line[1] || lsa->on_line[2];

    if (lsa->on_line[0] && !centre_and_right)
        return -1;
    if (lsa->on_line[4] && !centre_and_left)
        return 1;
    return 0;
}

int confirm(int dir, int *last_dir, int *count)
{
    if (dir != 0 && dir == *last_dir)
    {
        (*count)++;
    }
    else
    {
        *last_dir = dir;
        *count = dir ? 1 : 0;
    }
    return *count;
}

char node_plan_for(int node)
{
    const char *plan = NODE_PLAN;
    if (node >= 1 && node <= (int)strlen(plan))
    {
        return plan[node - 1];
    }
    return 'A';
}

exit_t choose_exit(bool left, bool straight, bool right, bool after_uturn, char plan)
{
    // Forced by NODE_PLAN, if that exit is really there
    if (plan == 'S' && straight)
        return EXIT_STRAIGHT;
    if (plan == 'L' && left)
        return EXIT_LEFT;
    if (plan == 'R' && right)
        return EXIT_RIGHT;

#if EXIT_RULE == RULE_RIGHT_HAND
    // Right wall follower: dead ends need no special handling
    (void)after_uturn;
    if (right)
        return EXIT_RIGHT;
    if (straight)
        return EXIT_STRAIGHT;
    if (left)
        return EXIT_LEFT;
    return EXIT_NONE;
#elif EXIT_RULE == RULE_LEFT_HAND
    (void)after_uturn;
    if (left)
        return EXIT_LEFT;
    if (straight)
        return EXIT_STRAIGHT;
    if (right)
        return EXIT_RIGHT;
    return EXIT_NONE;
#endif

    bool preferred = TURN_PREFERENCE < 0 ? left : right;
    bool other = TURN_PREFERENCE < 0 ? right : left;
    exit_t preferred_exit = TURN_PREFERENCE < 0 ? EXIT_LEFT : EXIT_RIGHT;
    exit_t other_exit = TURN_PREFERENCE < 0 ? EXIT_RIGHT : EXIT_LEFT;

    if (straight && !after_uturn)
        return EXIT_STRAIGHT;
    if (preferred)
        return preferred_exit;
    if (other)
        return other_exit;
    if (straight)
        return EXIT_STRAIGHT;
    return EXIT_NONE;
}

const char *exit_name(exit_t exit)
{
    switch (exit)
    {
    case EXIT_LEFT: return "LEFT";
    case EXIT_STRAIGHT: return "STRAIGHT";
    case EXIT_RIGHT: return "RIGHT";
    default: return "NONE";
    }
}
