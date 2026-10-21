/*
 * Node memory, see node_memory.h
 *
 * Direction maths, with the bot first arriving at the node heading "up":
 *
 *   took STRAIGHT, comes back heading down:  old LEFT is now on the right,
 *                                            old RIGHT on the left, the way it
 *                                            originally came is straight ahead
 *   took LEFT, comes back heading right:     old STRAIGHT is on the left,
 *                                            old RIGHT straight ahead, came from: right
 *   took RIGHT, comes back heading left:     old STRAIGHT is on the right,
 *                                            old LEFT straight ahead, came from: left
 */
#include "sra_board.h"
#include "config.h"
#include "node_memory.h"

static const char *TAG = "node memory";

#define MAX_NODES 16
#define EXIT_BIT(exit) (1 << (exit))

typedef struct
{
    uint8_t exits;      // exits seen on the first visit, EXIT_BIT(EXIT_LEFT) etc.
    uint8_t tried;      // exits already taken
    exit_t last_taken;  // the exit we are coming back along
} node_t;

static node_t stack[MAX_NODES];
static int depth = 0;

/*
 * Where an exit of the node (original direction) is, seen when coming back along 'taken'
 */
static exit_t seen_from_return(exit_t exit, exit_t taken)
{
    switch (taken)
    {
    case EXIT_STRAIGHT:
        return exit == EXIT_LEFT ? EXIT_RIGHT : exit == EXIT_RIGHT ? EXIT_LEFT : EXIT_NONE;
    case EXIT_LEFT:
        return exit == EXIT_STRAIGHT ? EXIT_LEFT : exit == EXIT_RIGHT ? EXIT_STRAIGHT : EXIT_NONE;
    case EXIT_RIGHT:
        return exit == EXIT_STRAIGHT ? EXIT_RIGHT : exit == EXIT_LEFT ? EXIT_STRAIGHT : EXIT_NONE;
    default:
        return EXIT_NONE;
    }
}

/*
 * The way the bot originally came into the node, seen when coming back along 'taken'
 */
static exit_t origin_from_return(exit_t taken)
{
    switch (taken)
    {
    case EXIT_STRAIGHT: return EXIT_STRAIGHT;
    case EXIT_LEFT: return EXIT_RIGHT;
    case EXIT_RIGHT: return EXIT_LEFT;
    default: return EXIT_NONE;
    }
}

static bool available(exit_t exit, bool left, bool straight, bool right)
{
    return (exit == EXIT_LEFT && left) || (exit == EXIT_STRAIGHT && straight) || (exit == EXIT_RIGHT && right);
}

void node_memory_reset(void)
{
    depth = 0;
}

void node_memory_push(bool left, bool straight, bool right, exit_t taken)
{
    if (taken == EXIT_NONE)
    {
        return;
    }
    if (depth == MAX_NODES)
    {
        // Full: forget the oldest node
        for (int i = 1; i < MAX_NODES; i++)
        {
            stack[i - 1] = stack[i];
        }
        depth--;
    }
    node_t *node = &stack[depth++];
    node->exits = (left ? EXIT_BIT(EXIT_LEFT) : 0) | (straight ? EXIT_BIT(EXIT_STRAIGHT) : 0) | (right ? EXIT_BIT(EXIT_RIGHT) : 0);
    node->tried = EXIT_BIT(taken);
    node->last_taken = taken;
}

exit_t node_memory_return(bool left, bool straight, bool right, bool *still_returning)
{
    *still_returning = false;
    if (depth == 0)
    {
        return EXIT_NONE;
    }
    node_t *node = &stack[depth - 1];

    // Check this really is the remembered node: from here we should see every other exit
    // it had plus the way we originally came. If not (the node was missed on the way in,
    // so this is a different junction), don't guess, let the caller use the normal rule.
    uint8_t expected = EXIT_BIT(origin_from_return(node->last_taken));
    for (exit_t exit = EXIT_LEFT; exit <= EXIT_RIGHT; exit++)
    {
        if ((node->exits & EXIT_BIT(exit)) && exit != node->last_taken)
        {
            expected |= EXIT_BIT(seen_from_return(exit, node->last_taken));
        }
    }
    uint8_t seen = (left ? EXIT_BIT(EXIT_LEFT) : 0) | (straight ? EXIT_BIT(EXIT_STRAIGHT) : 0) |
                   (right ? EXIT_BIT(EXIT_RIGHT) : 0);
    if (seen != expected)
    {
        ESP_LOGI(TAG, "this doesn't match the remembered node (seen %x, expected %x), using normal rule",
                 seen, expected);
        return EXIT_NONE;
    }

    // Untried exits, in the same order a new node would use
    exit_t preferred = TURN_PREFERENCE < 0 ? EXIT_LEFT : EXIT_RIGHT;
    exit_t other = TURN_PREFERENCE < 0 ? EXIT_RIGHT : EXIT_LEFT;
    exit_t order[3] = {EXIT_STRAIGHT, preferred, other};

    for (int i = 0; i < 3; i++)
    {
        exit_t exit = order[i];
        if (!(node->exits & EXIT_BIT(exit)) || (node->tried & EXIT_BIT(exit)))
        {
            continue;
        }
        exit_t now = seen_from_return(exit, node->last_taken);
        if (!available(now, left, straight, right))
        {
            continue;   // not seen this time, skip it rather than turn into nothing
        }
        ESP_LOGI(TAG, "back at node (depth %d): trying old %s = %s from here", depth, exit_name(exit), exit_name(now));
        node->tried |= EXIT_BIT(exit);
        node->last_taken = exit;
        return now;
    }

    // Every exit was a dead end: go back the way we originally came and check the node before
    exit_t back = origin_from_return(node->last_taken);
    depth--;
    *still_returning = true;
    ESP_LOGI(TAG, "node fully explored, heading back (%s), %d nodes left", exit_name(back), depth);
    return available(back, left, straight, right) ? back : EXIT_NONE;
}
