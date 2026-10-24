/*
 * Turn angle from the MPU-6050 z gyro, see gyro.h
 */
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "sra_board.h"
#include "i2cdev.h"
#include "config.h"
#include "gyro.h"

static const char *TAG = "gyro";

#define GYRO_RANGE_1000 0x10        // GYRO_CONFIG value for +-1000 deg/s
#define GYRO_LSB_PER_DPS 32.8f      // counts per deg/s at +-1000 deg/s. The library's +-250 deg/s
                                    // is less than a fast pivot, the reading would cap and under-count
#define BIAS_SAMPLES 100            // readings averaged at start-up

static bool found = false;
static float bias = 0;              // z reading while standing still, in counts
static float angle = 0;             // degrees turned since gyro_reset()
static int64_t last_us = 0;

/*
 * Raw z rate in counts, false if the read failed
 */
static bool read_z(float *z)
{
    uint8_t buf[BUFF_SIZE];
    int16_t raw[BUFF_SIZE / 2];
    if (mpu6050_read_gyro(buf, BUFF_SIZE) != ESP_OK)
        return false;
    combine_msb_lsb_raw_data(buf, raw);
    *z = raw[2];
    return true;
}

/*
 * The library's MPU device handle is private, so use our own one for the same chip
 */
static esp_err_t set_range_1000(void)
{
    i2c_dev_t dev = {0};
    dev.port = I2C_MASTER_NUM;
    dev.addr = MPU6050_ADDR;
    dev.cfg.mode = I2C_MODE_MASTER;
    dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
    dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;
    dev.cfg.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t err = i2c_dev_create_mutex(&dev);
    if (err != ESP_OK)
        return err;
    uint8_t value = GYRO_RANGE_1000;
    i2c_dev_take_mutex(&dev);
    err = i2c_dev_write_reg(&dev, MPU6050_RA_GYRO_CONFIG, &value, 1);
    i2c_dev_give_mutex(&dev);
    i2c_dev_delete_mutex(&dev);
    return err;
}

bool gyro_init(void)
{
    if (enable_mpu6050() != ESP_OK || set_range_1000() != ESP_OK)
    {
        ESP_LOGW(TAG, "MPU-6050 not found: turns use the arc fallback");
        return false;
    }
    float sum = 0, z;
    for (int i = 0; i < BIAS_SAMPLES; i++)
    {
        if (!read_z(&z))
        {
            ESP_LOGW(TAG, "MPU-6050 not answering: turns use the arc fallback");
            return false;
        }
        sum += z;
        vTaskDelay(pdMS_TO_TICKS(5));    // 100 readings over ~0.5 s
    }
    bias = sum / BIAS_SAMPLES;
    found = true;
    ESP_LOGI(TAG, "MPU-6050 ok, z bias %.1f counts", bias);
    return true;
}

bool gyro_ok(void)
{
    return found;
}

void gyro_reset(void)
{
    angle = 0;
    last_us = esp_timer_get_time();
}

float gyro_turned(void)
{
    float z;
    int64_t now = esp_timer_get_time();
    if (found && read_z(&z))
    {
        float dt = (now - last_us) / 1e6f;
        angle += (z - bias) / GYRO_LSB_PER_DPS * dt;
    }
    last_us = now;
    return fabsf(angle);
}
