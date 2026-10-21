/*
 * Maze solving for Wall-E: PID line following plus 90 / 45 degree turns,
 * junctions (nodes), dead ends and the end zone.
 *
 * Everything runs in maze_task() as a state machine. Each loop (LOOP_MS) it
 * reads the sensor bar once, then runs the code for the current state:
 *
 *   FOLLOW    normal PID line following. Watches for side branches, sharp
 *             turns and the line disappearing.
 *     -> APPROACH  side branch seen (11100 / 00111 / 11111)
 *     -> SHARP     line only under an outer sensor (10000 / 00001)
 *     -> UTURN     line vanished while it was centred (dead end)
 *     -> SEARCH    line vanished while it was off to one side
 *
 *   APPROACH  drive straight for APPROACH_MS so the wheels sit on the junction,
 *             then decide: end zone, go straight, or turn (choose_exit)
 *     -> DONE / FOLLOW / PIVOT / SEARCH
 *
 *   PIVOT     spin in place until the centre sensor lands on the new line -> FOLLOW
 *   SHARP     swing toward a 45 degree turn until the centre finds it    -> FOLLOW
 *   UTURN     spin ~180 degrees at a dead end                             -> FOLLOW
 *   SEARCH    sweep left / right, wider each time, until the line is back -> FOLLOW
 *   DONE      end zone reached, motors stopped
 *
 * Any turn that takes too long falls back to SEARCH.
 * The bar graph LEDs show the current state (see maze_state_t) and every state
 * change is logged on the serial monitor with the sensor pattern.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "line_sensor.h"
#include "motion.h"
#include "pid.h"

static const char *TAG = "maze";

/*
 * Line following speeds (same as the tuned 5_line_following example)
 */
#define BASE_DUTY 30
#define MIN_DUTY 25
#define MAX_DUTY 45

/*
 * 90 degree corners: line suddenly covers one side of the bar (11100 / 00111).
 * Drive forward until the axle is over the corner, then pivot in place onto the new line.
 */
#define CORNER_CONFIRM 3        // loops a side branch pattern must hold before we treat it as a corner / node
                                // (was 2: a wobbling line gave false junctions)
#define APPROACH_DUTY 40        // straight drive to bring the axle over the corner (was 50: drove past the corner)
#define APPROACH_MS 150         // depends on sensor bar to axle distance, tune on track
#define TURN_DUTY 45            // pivot speed (was 60, which motion.c caps to 50: overshot the line). Below 45 it stalls
#define PIVOT_MIN_MS 80         // ignore the line for this long after starting a pivot
#define PIVOT_TIMEOUT_MS 1500   // give up and search if the new line never shows
#define BRAKE_DUTY 50           // short reverse pulse to kill pivot overshoot
#define BRAKE_MS 30

/*
 * 45 degree turns: the line slides out to the outermost sensor faster than PID can follow
 * (10000 / 11000). Swing hard toward it, inner wheel backward, until the centre finds it.
 */
#define SHARP_CONFIRM 3         // was 2: a wobble out to the edge started a hard swing
#define SHARP_OUTER_DUTY 60     // wheel on the outside of the turn, forward
#define SHARP_INNER_DUTY 35     // wheel on the inside of the turn, backward
#define SHARP_TIMEOUT_MS 1000

/*
 * Nodes (junctions with 2+ exits). Straight always wins over a turn, except right after
 * a dead end U-turn, where straight would just take us back the way we came.
 */
#define TURN_PREFERENCE -1      // turn to take when there is no straight: -1 left, +1 right
/*
 * Optional per-node override, one character per node in the order they are counted:
 * 'A' = auto (rules above), 'S' = straight, 'L' = left, 'R' = right.
 * e.g. "AARA" forces a right turn at node 3. Nodes past the end of the string are auto.
 * If the forced exit doesn't exist at that node, auto is used.
 */
#define NODE_PLAN ""

/*
 * Dead end: line disappears while it was centred under the bar. Spin 180 degrees.
 */
#define UTURN_DIR TURN_PREFERENCE
#define UTURN_MIN_MS 400        // ignore the line for this long so we don't grab the stub we just left
#define UTURN_TIMEOUT_MS 3000

/*
 * Lost line recovery (gaps, acute turns sharper than 90)
 */
#define LOST_CONFIRM 3          // loops with no line before we start searching
#define DEADEND_CONFIRM 8       // loops with no line, driving straight, before a dead end U-turn
#define END_CONFIRM_MS 300      // end zone: bar must stay all white this much longer before stopping
#define START_ON_LINE_MS 500    // start: motors stay off until the bar has seen a line this long
#define RESUME_MS 1000          // DONE: put back on a line for this long and it carries on
#define SEARCH_DUTY 50
#define SEARCH_TIMEOUT_MS 800   // flip search direction after this, doubles each flip

#define LOOP_MS 10              // control loop period

/*
 * A side branch = centre sensor plus both sensors on that side
 */
#define LEFT_CORNER_MASK (LSA_BIT(0) | LSA_BIT(1) | LSA_BIT(2))
#define RIGHT_CORNER_MASK (LSA_BIT(2) | LSA_BIT(3) | LSA_BIT(4))

/*
 * Bar graph (only 4 LEDs usable while the motor driver is on) shows the state
 */
typedef enum
{
    STATE_FOLLOW = 0x1,
    STATE_APPROACH = 0x2,
    STATE_PIVOT = 0x4,
    STATE_SHARP = 0x6,
    STATE_SEARCH = 0x8,
    STATE_UTURN = 0xC,
    STATE_DONE = 0xF,
} maze_state_t;

static const char *state_name(maze_state_t state)
{
    switch (state)
    {
    case STATE_FOLLOW: return "FOLLOW";
    case STATE_APPROACH: return "APPROACH";
    case STATE_PIVOT: return "PIVOT";
    case STATE_SHARP: return "SHARP";
    case STATE_SEARCH: return "SEARCH";
    case STATE_UTURN: return "UTURN";
    case STATE_DONE: return "DONE";
    }
    return "?";
}

static uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static maze_state_t state = STATE_FOLLOW;
static uint32_t state_start_ms;     // when the current state started, for the timeouts

/*
 * Switches state: logs it, restarts the state timer and updates the bar graph
 */
static void set_state(maze_state_t next, const lsa_state_t *lsa)
{
    ESP_LOGI(TAG, "%s -> %s after %lu ms  pattern %d%d%d%d%d", state_name(state), state_name(next),
             (unsigned long)(now_ms() - state_start_ms),
             lsa->on_line[0], lsa->on_line[1], lsa->on_line[2], lsa->on_line[3], lsa->on_line[4]);
    state = next;
    state_start_ms = now_ms();
    set_bar_graph(next);
}

/*
 * Branch flags: a side branch shows up as the centre plus both sensors on that side
 * (11100 = left, 00111 = right, 11111 = both: T, cross or the end zone)
 */
#define BRANCH_LEFT 0x1
#define BRANCH_RIGHT 0x2

static int detect_branches(const lsa_state_t *lsa)
{
    int branches = 0;
    if ((lsa->pattern & LEFT_CORNER_MASK) == LEFT_CORNER_MASK)
        branches |= BRANCH_LEFT;
    if ((lsa->pattern & RIGHT_CORNER_MASK) == RIGHT_CORNER_MASK)
        branches |= BRANCH_RIGHT;
    return branches;
}

static int node_count = 0;              // junctions passed so far, shown on the OLED as "N: x"
static bool node_count_changed = true;  // redraw the OLED on the next update_display()

typedef enum
{
    EXIT_NONE,
    EXIT_LEFT,
    EXIT_STRAIGHT,
    EXIT_RIGHT,
} exit_t;

/*
 * Picks an exit at a node. Straight first, then TURN_PREFERENCE, then the other turn.
 * After a U-turn straight leads back to where we came from, so it is tried last.
 */
static exit_t choose_exit(bool left, bool straight, bool right, bool after_uturn, char plan)
{
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

static const char *exit_name(exit_t exit)
{
    switch (exit)
    {
    case EXIT_LEFT: return "LEFT";
    case EXIT_STRAIGHT: return "STRAIGHT";
    case EXIT_RIGHT: return "RIGHT";
    default: return "NONE";
    }
}

/*
 * 45 degree turn: line only under the outermost sensor(s) of one side.
 * Returns -1 for left, +1 for right, 0 otherwise
 */
static int detect_sharp(const lsa_state_t *lsa)
{
    // Only the outermost sensor on its own (10000 / 00001). 11000 / 00011 is just the line
    // a bit off centre, which the PID handles; swinging hard on it made following zigzag.
    if (lsa->pattern == LSA_BIT(0))
        return -1;
    if (lsa->pattern == LSA_BIT(4))
        return 1;
    return 0;
}

/*
 * Counts how many loops in a row the same non-zero direction was seen.
 * Used so one noisy reading can't trigger a turn: the caller only acts once
 * the count reaches CORNER_CONFIRM / SHARP_CONFIRM.
 */
static int confirm(int dir, int *last_dir, int *count)
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

/*
 * Redraws the OLED (Kp / Ki / Kd, IP, node count) only when something changed,
 * since drawing is slow. Only called from FOLLOW and DONE so it never delays a turn.
 */
static void update_display(void)
{
    pid_const_t k = read_pid_const();
    if (!k.val_changed && !node_count_changed)
    {
        return;
    }

    if (k.val_changed)
    {
        ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", k.kp, k.ki, k.kd);
    }
#ifdef CONFIG_ENABLE_OLED
    // Displaying kp, ki, kd values on OLED, node count in the top right corner
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    const char *ip_str_value = ip4addr_ntoa((const ip4_addr_t *)&ip_info.ip);
    display_pid_values(k.kp, k.ki, k.kd, ip_str_value);

    char node_str[12];
    snprintf(node_str, sizeof(node_str), "N: %d", node_count);
    lv_obj_t *node_label = lv_label_create(lv_disp_get_scr_act(NULL));
    lv_label_set_text(node_label, node_str);
    lv_obj_set_pos(node_label, 80, 0);
    lv_refr_now(NULL);
#endif
    reset_val_changed_pid_const();
    node_count_changed = false;
}

/*
 * Starts a 45 degree swing: outer wheel forward, inner wheel backward
 */
static void start_sharp(int dir)
{
    if (dir < 0)
        drive(-SHARP_INNER_DUTY, SHARP_OUTER_DUTY);
    else
        drive(SHARP_OUTER_DUTY, -SHARP_INNER_DUTY);
}

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

/*
 * Main loop, see the state list at the top of this file
 */
void maze_task(void *arg)
{
    ESP_ERROR_CHECK(motion_init());
    ESP_ERROR_CHECK(line_sensor_init());
    ESP_ERROR_CHECK(enable_bar_graph());
#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());
#endif

    lsa_state_t lsa;
    int node_seen = 0, node_confirm = 0;    // confirm() counters for side branches
    int sharp_dir = 0, sharp_count = 0;     // confirm() counters for 45 degree turns
    int lost_count = 0;                     // loops in a row with no line under the bar
    int turn_dir = 0;               // direction of the current turn / search, -1 left, +1 right
    int seen_branches = 0;          // side branches seen while driving over a node
    bool left_old_line = false;     // pivot: centre sensor has come off the line we turned from
    bool last_centred = false;      // line was in the middle of the bar the last time we saw it
                                    // (false at the start, so no line yet can't look like a dead end)
    bool after_uturn = false;       // last thing we did was a dead end U-turn
    uint32_t search_timeout = SEARCH_TIMEOUT_MS;
    const char *node_plan = NODE_PLAN;

    set_bar_graph(state);

    // Start: motors off until the bar has been on a line for START_ON_LINE_MS, so the bot
    // doesn't U-turn / search while it is in your hand or being put down
    stop();
    uint32_t line_since = 0;
    while (true)
    {
        line_sensor_update(&lsa);
        if (lsa.count == 0 || lsa.count == LSA_COUNT)
        {
            line_since = 0;
        }
        else if (line_since == 0)
        {
            line_since = now_ms();
        }
        else if (now_ms() - line_since >= START_ON_LINE_MS)
        {
            break;
        }
        update_display();
        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "on the line, starting");
    state_start_ms = now_ms();

    while (true)
    {
        line_sensor_update(&lsa);
        uint32_t elapsed = now_ms() - state_start_ms;

        switch (state)
        {
        /*
         * FOLLOW: checks run in priority order - junction, sharp turn, line lost,
         * and only if none of those fire, plain PID following
         */
        case STATE_FOLLOW:
        {
            update_display();

            // 1. Side branch held for CORNER_CONFIRM loops: drive onto the junction
            int branches = detect_branches(&lsa);
            if (confirm(branches ? 1 : 0, &node_seen, &node_confirm) >= CORNER_CONFIRM)
            {
                node_seen = node_confirm = 0;
                seen_branches = branches;
                set_state(STATE_APPROACH, &lsa);
                drive(APPROACH_DUTY, APPROACH_DUTY);
                break;
            }

            // 2. Line has slid out to an outer sensor: 45 degree turn
            if (confirm(detect_sharp(&lsa), &sharp_dir, &sharp_count) >= SHARP_CONFIRM)
            {
                turn_dir = sharp_dir;
                sharp_dir = sharp_count = 0;
                ESP_LOGI(TAG, "sharp %s turn", turn_dir < 0 ? "left" : "right");
                set_state(STATE_SHARP, &lsa);
                start_sharp(turn_dir);
                break;
            }

            // 3. No line at all: dead end or overshoot, decided by where the line was last seen
            if (lsa.count == 0)
            {
                lost_count++;
                if (last_centred)
                {
                    // Line ended while it was under the middle of the bar: drive straight,
                    // and only U-turn if it stays gone for DEADEND_CONFIRM loops (a gap or a
                    // thin line between two sensors comes back long before that)
                    drive(BASE_DUTY, BASE_DUTY);
                    if (lost_count >= DEADEND_CONFIRM)
                    {
                        lost_count = 0;
                        ESP_LOGI(TAG, "dead end, U-turn");
                        set_state(STATE_UTURN, &lsa);
                        pivot(UTURN_DIR, TURN_DUTY);
                    }
                    break;
                }
                if (lost_count >= LOST_CONFIRM)
                {
                    // Line slipped off one side: search toward the side it was last on
                    lost_count = 0;
                    turn_dir = pid_last_error() >= 0 ? 1 : -1;
                    search_timeout = SEARCH_TIMEOUT_MS;
                    set_state(STATE_SEARCH, &lsa);
                    pivot(turn_dir, SEARCH_DUTY);
                }
                // Brief gap: keep the last motor output going
                break;
            }
            lost_count = 0;
            // Remember if the line is under the middle (centre sensor on, outer ones off), used
            // for the dead end check above. Was "outer sensors off" only, so a line slipping off
            // the side through 01000 counted as centred and gave random U-turns.
            last_centred = lsa.on_line[2] && !lsa.on_line[0] && !lsa.on_line[4];

            // 4. Normal PID following: speed up one wheel and slow the other
            float correction = pid_compute(lsa.position);
            drive(bound(BASE_DUTY + correction, MIN_DUTY, MAX_DUTY),
                  bound(BASE_DUTY - correction, MIN_DUTY, MAX_DUTY));
            break;
        }

        /*
         * APPROACH: the sensor bar is ahead of the wheels, so drive forward until the
         * wheels are on the junction, then look at what is under the bar to decide
         */
        case STATE_APPROACH:
            // A cross can show 11100 first and 11111 a moment later, so keep collecting
            seen_branches |= detect_branches(&lsa);

            if (elapsed >= APPROACH_MS)
            {
                // Still white under the whole bar after driving onto it: maybe the end zone.
                // Keep driving END_CONFIRM_MS more and only stop if it is still all white, so a
                // wide junction or glare can't stop the run
                if (lsa.count == LSA_COUNT && elapsed < APPROACH_MS + END_CONFIRM_MS)
                {
                    break;
                }
                if (lsa.count == LSA_COUNT)
                {
                    stop();
                    ESP_LOGI(TAG, "END zone reached, %d nodes", node_count);
                    set_state(STATE_DONE, &lsa);
                    break;
                }

                // Exits available here. Straight = line still under the middle of the bar.
                bool left = seen_branches & BRANCH_LEFT;
                bool right = seen_branches & BRANCH_RIGHT;
                bool straight = lsa.on_line[1] || lsa.on_line[2] || lsa.on_line[3];
                // One exit = just a corner, two or more = a node (counted)
                bool is_node = left + right + straight >= 2;

                // Look up this node's entry in NODE_PLAN, 'A' (auto) if there is none
                char plan = 'A';
                if (is_node)
                {
                    node_count++;
                    node_count_changed = true;
                    if (node_count <= (int)strlen(node_plan))
                    {
                        plan = node_plan[node_count - 1];
                    }
                }

                exit_t choice = choose_exit(left, straight, right, after_uturn, plan);
                ESP_LOGI(TAG, "%s %d  exits L%d S%d R%d  plan %c  after_uturn %d -> %s",
                         is_node ? "node" : "corner", is_node ? node_count : 0,
                         left, straight, right, plan, after_uturn, exit_name(choice));
                if (is_node)
                {
                    after_uturn = false;
                }

                // Act on the choice: carry on, turn, or search if nothing was found
                if (choice == EXIT_STRAIGHT)
                {
                    pid_reset();
                    set_state(STATE_FOLLOW, &lsa);
                }
                else if (choice == EXIT_NONE)
                {
                    turn_dir = TURN_PREFERENCE;
                    search_timeout = SEARCH_TIMEOUT_MS;
                    set_state(STATE_SEARCH, &lsa);
                    pivot(turn_dir, SEARCH_DUTY);
                }
                else
                {
                    turn_dir = choice == EXIT_LEFT ? -1 : 1;
                    // If the straight line is still under the bar, the pivot has to leave it first
                    left_old_line = !lsa.on_line[2];
                    set_state(STATE_PIVOT, &lsa);
                    pivot(turn_dir, TURN_DUTY);
                }
            }
            break;

        /*
         * PIVOT: spin in place. First the centre sensor must leave the line we were on,
         * then the turn ends as soon as it lands on the next line.
         */
        case STATE_PIVOT:
            if (!lsa.on_line[2])
            {
                left_old_line = true;
            }

            // Pivot stops wherever the new line is, so this also lands on 45 degree exits
            if (left_old_line && elapsed >= PIVOT_MIN_MS && lsa.on_line[2])
            {
                finish_turn(-turn_dir);
                last_centred = true;
                set_state(STATE_FOLLOW, &lsa);
                break;
            }

            if (elapsed >= PIVOT_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "pivot timed out, searching");
                search_timeout = SEARCH_TIMEOUT_MS;
                set_state(STATE_SEARCH, &lsa);
                pivot(turn_dir, SEARCH_DUTY);
            }
            break;

        /*
         * UTURN: spin until the centre finds the line we came in on. after_uturn makes
         * the next node skip straight, which would lead back toward this dead end.
         */
        case STATE_UTURN:
            if (elapsed >= UTURN_MIN_MS && lsa.on_line[2])
            {
                finish_turn(-UTURN_DIR);
                after_uturn = true;
                last_centred = true;
                set_state(STATE_FOLLOW, &lsa);
                break;
            }

            if (elapsed >= UTURN_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "U-turn timed out, searching");
                turn_dir = UTURN_DIR;
                search_timeout = SEARCH_TIMEOUT_MS;
                set_state(STATE_SEARCH, &lsa);
                pivot(turn_dir, SEARCH_DUTY);
            }
            break;

        /*
         * SHARP: keep swinging (started by start_sharp) until the centre sensor is on the line
         */
        case STATE_SHARP:
            if (lsa.on_line[2])
            {
                pid_reset();
                last_centred = true;
                set_state(STATE_FOLLOW, &lsa);
                break;
            }

            if (elapsed >= SHARP_TIMEOUT_MS)
            {
                ESP_LOGW(TAG, "sharp turn timed out, searching");
                search_timeout = SEARCH_TIMEOUT_MS;
                set_state(STATE_SEARCH, &lsa);
                pivot(turn_dir, SEARCH_DUTY);
            }
            break;

        /*
         * SEARCH: last resort. Pivot one way, then back the other way for twice as long,
         * until any of the middle three sensors sees the line.
         */
        case STATE_SEARCH:
            if (lsa.on_line[1] || lsa.on_line[2] || lsa.on_line[3])
            {
                stop();
                pid_reset();
                last_centred = true;
                set_state(STATE_FOLLOW, &lsa);
                break;
            }

            if (elapsed >= search_timeout)
            {
                // Sweep back the other way, twice as far so we cover the first side again
                turn_dir = -turn_dir;
                search_timeout *= 2;
                state_start_ms = now_ms();
                ESP_LOGW(TAG, "search flip, now %s", turn_dir < 0 ? "left" : "right");
                pivot(turn_dir, SEARCH_DUTY);
            }
            break;

        // DONE: end zone, stay stopped. If it stopped by mistake, put it back on a line:
        // after RESUME_MS on a line (not all white) it carries on
        case STATE_DONE:
            stop();
            update_display();
            if (lsa.count == 0 || lsa.count == LSA_COUNT)
            {
                lost_count = 0;
            }
            else if (++lost_count * LOOP_MS >= RESUME_MS)
            {
                lost_count = 0;
                pid_reset();
                last_centred = true;
                ESP_LOGI(TAG, "put back on a line, resuming");
                set_state(STATE_FOLLOW, &lsa);
            }
            break;
        }

        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

/*
 * Starts WiFi + the PID tuning web page, then the maze task
 */
void app_main()
{
    start_tuning_http_server();
    xTaskCreate(&maze_task, "maze_task", 4096, NULL, 1, NULL);
}
