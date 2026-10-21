/*
 * 9_black_line: Wall-E follows the LEFT EDGE of the white line, black on its left,
 * white on its right. See bot.h for the states, config.h for every tunable.
 *
 * Why the edge: A0 and A1 sit on black the whole time, so white under them can only be
 * a line coming in from the left, never a wobble. And losing the white always means
 * "turn right" (right corner, dead end, or drifted left), so there are no U-turn guesses.
 *
 * This file: start up, the main loop, state switching and the OLED.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "bot.h"
#include "config.h"
#include "motion.h"

static const char *TAG = "bot";

uint32_t now_ms(void)
{
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint32_t elapsed(const bot_t *b)
{
    return now_ms() - b->state_ms;
}

static const char *state_name(bot_state_t s)
{
    switch (s)
    {
    case ST_FOLLOW: return "FOLLOW";
    case ST_APPROACH: return "APPROACH";
    case ST_TURN: return "TURN";
    case ST_WAIT: return "WAIT";
    case ST_DONE: return "DONE";
    }
    return "?";
}

void set_state(bot_t *b, bot_state_t next)
{
    const bool *on = b->lsa.on_line;
    ESP_LOGI(TAG, "%s -> %s after %lu ms  %d%d%d%d%d", state_name(b->state), state_name(next),
             (unsigned long)elapsed(b), on[0], on[1], on[2], on[3], on[4]);
    b->state = next;
    b->state_ms = now_ms();
    set_bar_graph(next);
}

/*
 * OLED: PID gains, IP (for the tuning page), nodes and U-turns. Its own low priority
 * task, so a slow redraw never holds up the loop that drives the motors.
 */
static void display_task(void *arg)
{
    bot_t *b = arg;
    while (true)
    {
        pid_const_t k = read_pid_const();
        if (k.val_changed || b->dirty)
        {
            b->dirty = false;
            reset_val_changed_pid_const();
#ifdef CONFIG_ENABLE_OLED
            esp_netif_ip_info_t ip;
            esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip);
            display_pid_values(k.kp, k.ki, k.kd, ip4addr_ntoa((const ip4_addr_t *)&ip.ip));
            char text[16];
            snprintf(text, sizeof(text), "N%d U%d", b->nodes, b->uturns);
            lv_obj_t *label = lv_label_create(lv_disp_get_scr_act(NULL));
            lv_label_set_text(label, text);
            lv_obj_set_pos(label, 80, 0);
            lv_refr_now(NULL);
#endif
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

static void bot_task(void *arg)
{
    ESP_ERROR_CHECK(motion_init());
    ESP_ERROR_CHECK(line_sensor_init());
    ESP_ERROR_CHECK(enable_bar_graph());
#ifdef CONFIG_ENABLE_OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);
    lv_obj_clean(lv_scr_act());
#endif

    // Lives as long as this task (the loop never ends), so the display task can read it
    bot_t b = {.state = ST_WAIT, .dirty = true};
    b.state_ms = now_ms();
    set_bar_graph(b.state);
    xTaskCreate(&display_task, "display", 4096, &b, 1, NULL);

    while (true)
    {
        line_sensor_update(&b.lsa);
        switch (b.state)
        {
        case ST_FOLLOW: follow(&b); break;
        case ST_APPROACH: approach(&b); break;
        case ST_TURN: turn(&b); break;
        case ST_WAIT:
        case ST_DONE: wait_for_line(&b); break;
        }
        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    start_tuning_http_server();
    xTaskCreate(&bot_task, "bot", 4096, NULL, 5, NULL);
}
