/*
 * IR sensor test (MH-B / FC-51 style module: IR LED + photodiode + LM393 comparator).
 *
 * Wiring: VCC -> 3V3, GND -> GND (header next to pin 36), OUT -> GPIO 27.
 *
 * The module's OUT is digital: the comparator flips it when the reflected IR crosses
 * the level set by the blue pot. Over a bright surface (white line) a lot of IR comes
 * back, OUT goes LOW and the "obstacle" LED on the module lights. Over black, OUT is HIGH.
 *
 * Every LOG_MS this prints the current reading, and every time it changes it prints the
 * change straight away with a count, so you can roll the bot across a few white lines
 * and check every line was seen exactly once.
 */
#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define IR_PIN GPIO_NUM_27
#define IR_WHITE_LEVEL 0    // OUT level over white. If white shows as BLACK in the log, set to 1
#define LOOP_MS 10          // read the pin this often
#define LOG_MS 500          // print the current reading this often, even if nothing changed

static const char *TAG = "IR";

void app_main(void)
{
    // OUT drives the pin itself (the module has its own pull-up), so no internal pull
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_LOGI(TAG, "reading GPIO %d, white = OUT %d. Move the sensor over white and black.",
             IR_PIN, IR_WHITE_LEVEL);

    int last_level = gpio_get_level(IR_PIN);
    int white_count = 0;    // white patches seen (black -> white changes)
    TickType_t last_log = xTaskGetTickCount();

    while (1)
    {
        int level = gpio_get_level(IR_PIN);
        bool white = level == IR_WHITE_LEVEL;

        if (level != last_level)
        {
            if (white)
            {
                white_count++;
            }
            ESP_LOGI(TAG, "changed -> %s  (OUT %d)  white patches so far: %d",
                     white ? "WHITE" : "BLACK", level, white_count);
            last_level = level;
        }

        if ((xTaskGetTickCount() - last_log) * portTICK_PERIOD_MS >= LOG_MS)
        {
            last_log = xTaskGetTickCount();
            printf("IR: %s  (OUT %d)\n", white ? "WHITE" : "black", level);
        }

        vTaskDelay(LOOP_MS / portTICK_PERIOD_MS);
    }
}
