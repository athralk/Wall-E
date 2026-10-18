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
#include "backtrack.h"

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
    case STATE_PIVOT: return "PIVOT";
    case STATE_SHARP: return "SHARP";
    case STATE_SEARCH: return "SEARCH";
    case STATE_DROP: return "DROP";
    case STATE_BACKTRACK: return "BACKTRACK";
    case STATE_UTURN: return "UTURN";
    case STATE_DONE: return "DONE";
    }
    return "?";
}

void set_state(maze_ctx_t *ctx, maze_state_t next)
{
    const lsa_state_t *lsa = &ctx->lsa;
    ESP_LOGI(TAG, "%s -> %s after %lu ms  pattern %d%d%d%d%d", state_name(ctx->state), state_name(next),
             (unsigned long)state_elapsed(ctx),
             lsa->on_line[0], lsa->on_line[1], lsa->on_line[2], lsa->on_line[3], lsa->on_line[4]);
    ctx->state = next;
    ctx->state_start_ms = now_ms();
    set_bar_graph(next);
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

    // last_centred starts false: if the line can't be found at the start, search for it
    // instead of treating it as a dead end
    maze_ctx_t ctx = {
        .state = STATE_FOLLOW,
        .last_centred = false,
        .search_timeout = SEARCH_TIMEOUT_MS,
        .display_dirty = true,
    };
    set_bar_graph(ctx.state);
    ctx.state_start_ms = now_ms();
    ctx.hold_until_ms = now_ms() + START_HOLD_MS;

    while (true)
    {
        line_sensor_update(&ctx.lsa);
        log_readings(&ctx.lsa);

        // Colour flips only make sense while driving along a line, not mid turn
        if (ctx.state == STATE_FOLLOW || ctx.state == STATE_APPROACH)
        {
            if (colour_blind_update(&ctx) && ctx.state == STATE_APPROACH)
            {
                // The white edge of the block looked like a junction: it wasn't, carry on
                back_to_follow(&ctx);
            }
        }

        switch (ctx.state)
        {
        case STATE_FOLLOW: handle_follow(&ctx); break;
        case STATE_APPROACH: handle_approach(&ctx); break;
        case STATE_PIVOT: handle_pivot(&ctx); break;
        case STATE_SHARP: handle_sharp(&ctx); break;
        case STATE_UTURN: handle_uturn(&ctx); break;
        case STATE_SEARCH: handle_search(&ctx); break;
        case STATE_DROP: handle_drop(&ctx); break;
        case STATE_BACKTRACK: handle_backtrack(&ctx); break;
        case STATE_DONE: handle_done(&ctx); break;
        }

        // Remember how the wheels were driven this loop, so BACKTRACK can undo it
        if (ctx.state != STATE_BACKTRACK && ctx.state != STATE_DONE && ctx.state != STATE_DROP)
        {
            backtrack_record();
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
