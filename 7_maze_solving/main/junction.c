/*
 * Junction / turn pattern checks and the exit choice, see junction.h
 */
#include <string.h>
#include "config.h"
#include "junction.h"

/*
 * A side branch = centre sensor plus both sensors on that side
 */
#define LEFT_CORNER_MASK (LSA_BIT(0) | LSA_BIT(1) | LSA_BIT(2))
#define RIGHT_CORNER_MASK (LSA_BIT(2) | LSA_BIT(3) | LSA_BIT(4))

int detect_branches(const lsa_state_t *lsa)
{
    int branches = 0;
    if ((lsa->pattern & LEFT_CORNER_MASK) == LEFT_CORNER_MASK)
        branches |= BRANCH_LEFT;
    if ((lsa->pattern & RIGHT_CORNER_MASK) == RIGHT_CORNER_MASK)
        branches |= BRANCH_RIGHT;
    return branches;
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
