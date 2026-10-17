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
#define BASE_DUTY 35
#define MIN_DUTY 25
#define MAX_DUTY 50

/*
 * 90 degree corners: line suddenly covers one side of the bar (11100 / 00111).
 * Drive forward until the axle is over the corner, then pivot in place onto the new line.
 */
#define CORNER_CONFIRM 2        // loops a side branch pattern must hold before we treat it as a corner / node
#define APPROACH_DUTY 50        // straight drive to bring the axle over the corner
#define APPROACH_MS 150         // depends on sensor bar to axle distance, tune on track
#define TURN_DUTY 60            // pivot speed
#define PIVOT_MIN_MS 80         // ignore the line for this long after starting a pivot
#define PIVOT_TIMEOUT_MS 1500   // give up and search if the new line never shows
#define BRAKE_DUTY 50           // short reverse pulse to kill pivot overshoot
#define BRAKE_MS 30

/*
 * 45 degree turns: the line slides out to the outermost sensor faster than PID can follow
 * (10000 / 11000). Swing hard toward it, inner wheel backward, until the centre finds it.
 */
#define SHARP_CONFIRM 2
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
#define SEARCH_DUTY 50
#define SEARCH_TIMEOUT_MS 800   // flip search direction after this, doubles each flip

#define LOOP_MS 10

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
static uint32_t state_start_ms;

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

static int node_count = 0;
static bool node_count_changed = true;

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
    bool centre_and_right = lsa->on_line[2] || lsa->on_line[3] || lsa->on_line[4];
    bool centre_and_left = lsa->on_line[0] || lsa->on_line[1] || lsa->on_line[2];

    if (lsa->on_line[0] && !centre_and_right)
        return -1;
    if (lsa->on_line[4] && !centre_and_left)
        return 1;
    return 0;
}

/*
 * Counts how many loops in a row the same non-zero direction was seen
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

static void start_sharp(int dir)
{
    if (dir < 0)
        drive(-SHARP_INNER_DUTY, SHARP_OUTER_DUTY);
    else
        drive(SHARP_OUTER_DUTY, -SHARP_INNER_DUTY);
}

static void finish_turn(int brake_dir)
{
    pivot(brake_dir, BRAKE_DUTY);
    vTaskDelay(BRAKE_MS / portTICK_PERIOD_MS);
    stop();
    pid_reset();
}

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
    int node_seen = 0, node_confirm = 0;
    int sharp_dir = 0, sharp_count = 0;
    int lost_count = 0;
    int turn_dir = 0;               // direction of the current turn / search, -1 left, +1 right
    int seen_branches = 0;          // side branches seen while driving over a node
    bool left_old_line = false;     // pivot: centre sensor has come off the line we turned from
    bool last_centred = true;       // line was in the middle of the bar the last time we saw it
    bool after_uturn = false;       // last thing we did was a dead end U-turn
    uint32_t search_timeout = SEARCH_TIMEOUT_MS;
    const char *node_plan = NODE_PLAN;

    set_bar_graph(state);
    state_start_ms = now_ms();

    while (true)
    {
        line_sensor_update(&lsa);
        uint32_t elapsed = now_ms() - state_start_ms;

        switch (state)
        {
        case STATE_FOLLOW:
        {
            update_display();

            int branches = detect_branches(&lsa);
            if (confirm(branches ? 1 : 0, &node_seen, &node_confirm) >= CORNER_CONFIRM)
            {
                node_seen = node_confirm = 0;
                seen_branches = branches;
                set_state(STATE_APPROACH, &lsa);
                drive(APPROACH_DUTY, APPROACH_DUTY);
                break;
            }

            if (confirm(detect_sharp(&lsa), &sharp_dir, &sharp_count) >= SHARP_CONFIRM)
            {
                turn_dir = sharp_dir;
                sharp_dir = sharp_count = 0;
                ESP_LOGI(TAG, "sharp %s turn", turn_dir < 0 ? "left" : "right");
                set_state(STATE_SHARP, &lsa);
                start_sharp(turn_dir);
                break;
            }

            if (lsa.count == 0)
            {
                if (++lost_count >= LOST_CONFIRM)
                {
                    lost_count = 0;
                    if (last_centred)
                    {
                        // Line ended while it was under the middle of the bar: dead end
                        ESP_LOGI(TAG, "dead end, U-turn");
                        set_state(STATE_UTURN, &lsa);
                        pivot(UTURN_DIR, TURN_DUTY);
                        break;
                    }
                    turn_dir = pid_last_error() >= 0 ? 1 : -1;
                    search_timeout = SEARCH_TIMEOUT_MS;
                    set_state(STATE_SEARCH, &lsa);
                    pivot(turn_dir, SEARCH_DUTY);
                }
                // Brief gap: keep the last motor output going
                break;
            }
            lost_count = 0;
            last_centred = !lsa.on_line[0] && !lsa.on_line[4];

            float correction = pid_compute(lsa.position);
            drive(bound(BASE_DUTY + correction, MIN_DUTY, MAX_DUTY),
                  bound(BASE_DUTY - correction, MIN_DUTY, MAX_DUTY));
            break;
        }

        case STATE_APPROACH:
            // A cross can show 11100 first and 11111 a moment later, so keep collecting
            seen_branches |= detect_branches(&lsa);

            if (elapsed >= APPROACH_MS)
            {
                // Still white under the whole bar after driving onto it: the end zone
                if (lsa.count == LSA_COUNT)
                {
                    stop();
                    ESP_LOGI(TAG, "END zone reached, %d nodes", node_count);
                    set_state(STATE_DONE, &lsa);
                    break;
                }

                bool left = seen_branches & BRANCH_LEFT;
                bool right = seen_branches & BRANCH_RIGHT;
                bool straight = lsa.on_line[1] || lsa.on_line[2] || lsa.on_line[3];
                bool is_node = left + right + straight >= 2;

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

        case STATE_DONE:
            stop();
            update_display();
            break;
        }

        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    start_tuning_http_server();
    xTaskCreate(&maze_task, "maze_task", 4096, NULL, 1, NULL);
}
