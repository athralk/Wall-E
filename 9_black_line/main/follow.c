/*
 * FOLLOW and APPROACH.
 *
 * Centre mode (normal line following): PID on the centre of the line. When the line
 * vanishes, turn toward the side an outer sensor last saw it (corner), left if both
 * (a T), right round the end of the line if neither (dead end). Straight wins at a cross.
 *
 * Edge mode (after EDGE_AFTER_MS): PID on the left edge of the line. White under A0 + A1
 * is a line from the left -> APPROACH, then turn left. Line gone -> turn right.
 * That is the left hand rule: left, else straight, else right, else U-turn.
 */
#include "sra_board.h"
#include "bot.h"
#include "config.h"
#include "edge.h"
#include "motion.h"
#include "pid.h"

static const char *TAG = "bot";

bool edge_mode(const bot_t *b)
{
#if EDGE_AFTER_MS > 0
    return b->run_ms && now_ms() - b->run_ms >= EDGE_AFTER_MS;
#else
    (void)b;
    return false;   // centre the whole run
#endif
}

void start_follow(bot_t *b)
{
    pid_reset();
    b->lost_count = 0;
    b->creep_until = 0;
    b->left_seen = b->right_seen = 0;
    b->last_first = EDGE_TARGET;
    b->left_count = 0;
    b->armed = false;
    b->white_since = 0;
    set_state(b, ST_FOLLOW);
}

static void drive_pid(bot_t *b, uint32_t t, float error)
{
    bool kick = t < b->kick_until;
    int base = kick ? KICK_DUTY : BASE_DUTY;
    int low = kick ? KICK_MIN_DUTY : MIN_DUTY;
    float c = pid_compute(error);
    drive(bound(base + c, low, MAX_DUTY), bound(base - c, low, MAX_DUTY));
}

/*
 * Centre mode: which way the line went. -1 left, +1 right, 0 dead end.
 */
static int centre_turn_side(const bot_t *b, uint32_t t)
{
    bool l = b->left_seen && t - b->left_seen < SIDE_MEMORY_MS;
    bool r = b->right_seen && t - b->right_seen < SIDE_MEMORY_MS;
    if (l && r)     // both: the later one, or left if they came together (a T)
        return b->right_seen > b->left_seen + SIDE_SAME_MS ? 1 : -1;
    return l ? -1 : r ? 1 : 0;
}

/*
 * No line under the bar for LOST_CONFIRM loops: roll on CREEP_MS so the wheels reach
 * the corner, then turn. Edge mode after a slow drift (... 00001): turn at once.
 */
static void line_lost(bot_t *b, uint32_t t)
{
    if (++b->lost_count < LOST_CONFIRM)
        return;     // one bad reading: keep the last motor output
    bool edge = edge_mode(b);
    int dir = edge ? 1 : centre_turn_side(b, t);
    bool creep = !edge || b->last_first <= EDGE_TARGET + 1;
    if (creep && b->creep_until == 0)
        b->creep_until = t + CREEP_MS;
    if (creep && t < b->creep_until)
    {
        drive(CREEP_DUTY, CREEP_DUTY);
        return;
    }
    if (dir == 0)
    {
        ESP_LOGI(TAG, "dead end: U-turn");
        start_turn(b, 1);
        b->uturn = true;
        b->uturns++;
        b->dirty = true;
        return;
    }
    start_turn(b, dir);
}

static void follow_centre(bot_t *b, uint32_t t)
{
    if (b->lsa.on_line[0])
        b->left_seen = t;
    if (b->lsa.on_line[LSA_COUNT - 1])
        b->right_seen = t;
    drive_pid(b, t, centre_error(&b->lsa));
}

static void follow_edge(bot_t *b, uint32_t t)
{
    const lsa_state_t *lsa = &b->lsa;
    // Left branches only count once the edge has been found since the last turn
    if (!lsa->on_line[0] && !lsa->on_line[1])
        b->armed = true;
    bool left = b->armed && edge_left_branch(lsa);
    b->left_count = left ? b->left_count + 1 : 0;
    if (b->left_count >= LEFT_CONFIRM)
    {
        set_state(b, ST_APPROACH);
        drive(APPROACH_DUTY, APPROACH_DUTY);
        return;
    }
    if (b->left_count > 0)
    {
        drive(BASE_DUTY, BASE_DUTY);    // maybe a branch: don't swing into it yet
        return;
    }
    b->last_first = edge_first_white(lsa);
    drive_pid(b, t, edge_error(lsa));
}

void follow(bot_t *b)
{
    uint32_t t = now_ms();
    if (b->lsa.count == 0)
    {
        line_lost(b, t);
        return;
    }
    b->lost_count = 0;
    b->creep_until = 0;

    // Whole bar white for END_WHITE_MS: the end zone
    if (b->lsa.count < LSA_COUNT)
        b->white_since = 0;
    else if (b->white_since == 0)
        b->white_since = t;
    if (b->white_since && t - b->white_since >= END_WHITE_MS)
    {
        stop();
        ESP_LOGI(TAG, "END zone: %d nodes, %d U-turns", b->nodes, b->uturns);
        b->line_since = 0;
        set_state(b, ST_DONE);
        return;
    }

    bool edge = edge_mode(b);
    if (edge && !b->edge_was)
    {
        ESP_LOGI(TAG, "switching to the left edge");
        pid_reset();
    }
    b->edge_was = edge;
    if (edge)
        follow_edge(b, t);
    else
        follow_centre(b, t);
}

void approach(bot_t *b)
{
    if (elapsed(b) < APPROACH_MS)
        return;
    // Still all white: a wide white patch, not a turn. FOLLOW steers onto the edge.
    if (b->lsa.count == LSA_COUNT)
    {
        ESP_LOGI(TAG, "all white: steering onto the left edge");
        start_follow(b);
        return;
    }
    // Line still ahead too = a node (left + straight), else just a left corner
    if (b->lsa.count > 0)
    {
        b->nodes++;
        b->dirty = true;
        ESP_LOGI(TAG, "node %d: left and straight, going left", b->nodes);
    }
    start_turn(b, -1);
}
