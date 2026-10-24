/*
 * OLED display, see display.h
 */
#include <stdio.h>
#include "sra_board.h"
#include "tuning_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"
#include "display.h"
#include "line_sensor.h"

void display_update(maze_ctx_t *ctx)
{
    pid_const_t k = read_pid_const();
    if (!k.val_changed && !ctx->display_dirty)
    {
        return;
    }

    if (k.val_changed)
    {
        ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", k.kp, k.ki, k.kd);
    }
#ifdef CONFIG_ENABLE_OLED
    // Displaying kp, ki, kd values on OLED
    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    const char *ip_str_value = ip4addr_ntoa((const ip4_addr_t *)&ip_info.ip);
    display_pid_values(k.kp, k.ki, k.kd, ip_str_value);

    // Node count (and INV in the colour blind section) in the top right corner
    char status[16];
    snprintf(status, sizeof(status), "N: %d%s", ctx->node_count, line_sensor_is_inverted() ? " INV" : "");
    lv_obj_t *status_label = lv_label_create(lv_disp_get_scr_act(NULL));
    lv_label_set_text(status_label, status);
    lv_obj_set_pos(status_label, 70, 0);
    lv_refr_now(NULL);
#endif
    reset_val_changed_pid_const();
    ctx->display_dirty = false;
}

/*
 * Screen layout:
 *   S1-S3: 950  640  960
 *   S4-S5: 610  600
 *   10100  (1 = sees the line)
 */
void display_readings(const lsa_state_t *lsa)
{
    const int *r = lsa->reading;
    ESP_LOGI("lsa", "%4d %4d %4d %4d %4d   %d%d%d%d%d", r[0], r[1], r[2], r[3], r[4],
             lsa->on_line[0], lsa->on_line[1], lsa->on_line[2], lsa->on_line[3], lsa->on_line[4]);
#ifdef CONFIG_ENABLE_OLED
    char lines[3][24];
    snprintf(lines[0], sizeof(lines[0]), "1-3: %d %d %d", r[0], r[1], r[2]);
    snprintf(lines[1], sizeof(lines[1]), "4-5: %d %d", r[3], r[4]);
    snprintf(lines[2], sizeof(lines[2]), "%d%d%d%d%d", lsa->on_line[0], lsa->on_line[1],
             lsa->on_line[2], lsa->on_line[3], lsa->on_line[4]);

    lv_obj_clean(lv_scr_act());
    for (int i = 0; i < 3; i++)
    {
        lv_obj_t *label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, lines[i]);
        lv_obj_set_pos(label, 0, i * 20);
    }
    lv_refr_now(NULL);
#endif
}
