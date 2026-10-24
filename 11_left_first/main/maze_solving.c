/*
 * Maze solving for Wall-E (SAC 2026): PID line following plus 90 / 45 degree
 * turns, nodes, dead ends, the colour blind section, the pink drop zone and
 * the end zone.
 *
 * This file only starts things and runs the loop. Each loop it:
 *   1. reads the sensor bar
 *   2. lets colour_blind.c flip the polarity if we entered / left the black-on-white block
 *   3. runs the handler for the current state (see maze.h for which file has which)
 *
 * All tunables are in include/config.h.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "config.h"
#include "maze.h"
#include "line_sensor.h"
#include "motion.h"
#include "colour_blind.h"
#include "turn_90.h"
#include "turn_120.h"
#include "display.h"

static const char *TAG = "maze";

uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint32_t state_elapsed(const maze_ctx_t *ctx)
{
    return now_ms() - ctx->state_start_ms;
}

static const char *state_name(maze_state_t state)
{
    switch (state)
    {
    case STATE_FOLLOW: return "FOLLOW";
    case STATE_APPROACH: return "APPROACH";
    case STATE_TURN_90: return "TURN_90";
    case STATE_TURN_120: return "TURN_120";
    case STATE_SEARCH: return "SEARCH";
    case STATE_DROP: return "DROP";
    case STATE_OBJECT_DETECTED: return "Object_detected";
    case STATE_DONE: return "DONE";
    }
    return "?";
}

static const char *live_state = "FOLLOW";     // for the live LSA web page

const char *maze_state_text(void)
{
    return live_state;
}

void set_state(maze_ctx_t *ctx, maze_state_t next)
{
    live_state = state_name(next);
    const lsa_state_t *lsa = &ctx->lsa;
    ESP_LOGI(TAG, "%s -> %s after %lu ms  pattern %d%d%d%d%d  front IR %d", state_name(ctx->state), state_name(next),
             (unsigned long)state_elapsed(ctx),
             lsa->on_line[0], lsa->on_line[1], lsa->on_line[2], lsa->on_line[3], lsa->on_line[4], lsa->ir_object_detected);
    ctx->state = next;
    ctx->state_start_ms = now_ms();
}

/*
 * Prints the raw-ish readings so you can find the pink band (LOG_READINGS in config.h)
 */
static void log_readings(const lsa_state_t *lsa)
{
#if LOG_READINGS
    static uint32_t last_ms = 0;
    if (now_ms() - last_ms >= LOG_READINGS_MS)
    {
        last_ms = now_ms();
        ESP_LOGI("lsa", "%4d %4d %4d %4d %4d", lsa->reading[0], lsa->reading[1],
                 lsa->reading[2], lsa->reading[3], lsa->reading[4]);
    }
#endif
}

/*
 * Redraws the OLED when needed. Its own task at a lower priority than maze_task, so the
 * slow I2C redraw never sits between reading the LSA and driving the motors.
 */
static void display_task(void *arg)
{
    maze_ctx_t *ctx = arg;
    while (true)
    {
        display_update(ctx);
        vTaskDelay(DISPLAY_MS / portTICK_PERIOD_MS);
    }
}

void maze_task(void *arg)
{
    ESP_ERROR_CHECK(motion_init());
    ESP_ERROR_CHECK(line_sensor_init());
#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());
#endif

#if CALIBRATE_LSA
    // Calibration mode: motors off, readings on the OLED, never runs the maze
    stop();
    while (true)
    {
        lsa_state_t lsa;
        line_sensor_update(&lsa);
        display_readings(&lsa);
        vTaskDelay(LOG_READINGS_MS / portTICK_PERIOD_MS);
    }
#endif

    // last_centred starts false: if the line can't be found at the start, search for it
    // instead of treating it as a dead end
    maze_ctx_t ctx = {
        .state = STATE_FOLLOW,
        .last_centred = false,
        .search_timeout = SEARCH_TIMEOUT_MS,
        .display_dirty = true,
    };
    ctx.state_start_ms = now_ms();
    ctx.hold_until_ms = now_ms() + START_HOLD_MS;
    ctx.kick_until_ms = now_ms() + KICK_MS;
    // ctx lives as long as this task (the loop below never ends), so the display can read it
    xTaskCreate(&display_task, "display_task", 4096, &ctx, 1, NULL);

    while (true)
    {
        line_sensor_update(&ctx.lsa);
        log_readings(&ctx.lsa);

        // A front obstacle is a U-turn trigger only when the entire LSA also reads
        // raw white. Check before FOLLOW/APPROACH so startup and end-zone handling
        // cannot return or stop first. Either sensor condition by itself does nothing.
        bool object_and_all_white = ctx.lsa.ir_object_detected &&
                                    ctx.lsa.raw_white_count == LSA_COUNT;
        bool can_handle_object = ctx.state == STATE_FOLLOW || ctx.state == STATE_APPROACH;
        if (can_handle_object && object_and_all_white)
        {
            ctx.object_confirm_count++;
            if (ctx.object_confirm_count >= OBJECT_CONFIRM_LOOPS)
            {
                ctx.object_confirm_count = 0;
                ESP_LOGI(TAG, "front object + all-white LSA; entering Object_detected");
                set_state(&ctx, STATE_OBJECT_DETECTED);
                handle_object_detected(&ctx);
                vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
                continue;
            }
        }
        else
        {
            ctx.object_confirm_count = 0;
        }

        // Keep polarity changes in the line-following states. If the colour changes
        // while approaching a node, restart FOLLOW with fresh line-width/PID state.
        if (ctx.state == STATE_FOLLOW || ctx.state == STATE_APPROACH)
        {
            if (colour_blind_update(&ctx) && ctx.state == STATE_APPROACH)
            {
                back_to_follow(&ctx);
            }
        }

        // Confirm the end zone from the LSA directly, so a missed branch pattern
        // cannot send the robot past the white finish area. Ignore the start patch
        // and the white background while following the inverted black line.
        static int end_white_count = 0;
        bool end_zone_candidate = ctx.seen_line && !ctx.leaving_start &&
                                  (ctx.state == STATE_FOLLOW || ctx.state == STATE_APPROACH) &&
                                  !line_sensor_is_inverted() &&
                                  ctx.lsa.raw_white_count == LSA_COUNT;
        end_white_count = end_zone_candidate ? end_white_count + 1 : 0;
        int end_confirm_loops = (END_CONFIRM_MS + LOOP_MS - 1) / LOOP_MS;
        if (end_confirm_loops < 1)
            end_confirm_loops = 1;
        if (end_white_count >= end_confirm_loops)
        {
            end_white_count = 0;
            ctx.line_seen_since_ms = 0;
            stop();
            ESP_LOGI(TAG, "END zone reached, %d nodes", ctx.node_count);
            set_state(&ctx, STATE_DONE);
        }

        switch (ctx.state)
        {
        case STATE_FOLLOW: handle_follow(&ctx); break;
        case STATE_APPROACH: handle_approach(&ctx); break;
        case STATE_TURN_90: handle_turn_90(&ctx); break;
        case STATE_TURN_120: handle_turn_120(&ctx); break;
        case STATE_SEARCH: handle_search(&ctx); break;
        case STATE_DROP: break;    // pink zone off
        case STATE_OBJECT_DETECTED: handle_object_detected(&ctx); break;
        case STATE_DONE: handle_done(&ctx); break;
        }

        // Always sleep at least one tick, so the idle task, Wi-Fi and the OLED get CPU time
        TickType_t ticks = pdMS_TO_TICKS(LOOP_MS);
        vTaskDelay(ticks > 0 ? ticks : 1);
    }

    vTaskDelete(NULL);
}

/*
 * Starts WiFi + the PID tuning web page, then the maze task
 */
void app_main()
{
    start_tuning_http_server();
    // Above display_task (1) so an OLED redraw never delays the control loop
    xTaskCreate(&maze_task, "maze_task", 4096, NULL, 5, NULL);
}
