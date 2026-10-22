#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include "esp_netif.h"
#include "lwip/ip4_addr.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY  820    // Boundary value to distinguish between black and white readings

/*
 * 90 degree turn parameters
 */
#define TURN_CONFIRM_COUNT 3     // consecutive readings (x10 ms) the turn pattern must hold
#define CREEP_MS           120   // drive forward so the wheel axle reaches the corner (LSA is ahead of wheels)
#define TURN_DUTY          45    // duty cycle while pivoting in place
#define TURN_TIMEOUT_MS    2000  // give up pivoting if the line is not found

/*
 * weights given to respective line sensor
 */
const int weights[5] = {-5, -3, 1, 3, 5};

/*
 * Motor value boundts
 */
int optimum_duty_cycle = 38;
int lower_duty_cycle = 30;
int higher_duty_cycle = 47;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error=0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

/*
 * on_line[i] = 1 if sensor i sees the line (reading > BLACK_BOUNDARY)
 */
int on_line[5];

void read_sensors(adc_handle_t line_sensor)
{
    line_sensor_readings = read_line_sensor(line_sensor);
    for(int i = 0; i < 5; i++)
    {
        line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
        line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        on_line[i] = line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY;
    }
}

void calculate_correction()
{
    error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0; 
    float pos = 0; int k = 0;

    for(int i = 0; i < 5; i++)
    {
        if(on_line[i])
        {
            all_black_flag = 0;
        }
        k = on_line[i];
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if(sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if(all_black_flag == 1)  // If all black then we check for previous error to assign current error.
    {
        if(prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

/*
 * Returns -1 for a left 90 degree turn, +1 for a right one, 0 otherwise.
 * Sensor 0 is on the left, sensor 4 on the right.
 */
int detect_turn()
{
    if(on_line[2] && on_line[3] && on_line[4] && !on_line[0])
    {
        return 1;
    }
    if(on_line[2] && on_line[1] && on_line[0] && !on_line[4])
    {
        return -1;
    }
    return 0;
}

void take_turn(motor_handle_t motor_a_0, motor_handle_t motor_a_1, adc_handle_t line_sensor, int dir)
{
    // roll forward so the wheel axle is over the corner
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(CREEP_MS / portTICK_PERIOD_MS);

    // pivot in place: one wheel forward, the other backward
    set_motor_speed(motor_a_0, dir > 0 ? MOTOR_FORWARD : MOTOR_BACKWARD, TURN_DUTY);
    set_motor_speed(motor_a_1, dir > 0 ? MOTOR_BACKWARD : MOTOR_FORWARD, TURN_DUTY);

    // centre sensor must leave the old line first, then land on the new one
    int left_old_line = 0;
    TickType_t start = xTaskGetTickCount();
    while((xTaskGetTickCount() - start) * portTICK_PERIOD_MS < TURN_TIMEOUT_MS)
    {
        read_sensors(line_sensor);
        if(!on_line[2])
        {
            left_old_line = 1;
        }
        else if(left_old_line)
        {
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // clear PID memory so the first correction after the turn doesn't kick
    cumulative_error = 0;
    prev_error = 0;
}

void line_follow_task(void* arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());
#ifdef CONFIG_ENABLE_OLED
    // Initialising the OLED
    ESP_ERROR_CHECK(init_oled());
    vTaskDelay(100);

    // Clearing the screen
    lv_obj_clean(lv_scr_act());

#endif

    int turn_count = 0, last_turn = 0;

    while(true)
    {
        read_sensors(line_sensor);

        // a turn pattern must hold for TURN_CONFIRM_COUNT readings in a row
        int turn = detect_turn();
        turn_count = (turn != 0 && turn == last_turn) ? turn_count + 1 : (turn != 0);
        last_turn = turn;
        if(turn_count >= TURN_CONFIRM_COUNT)
        {
            take_turn(motor_a_0, motor_a_1, line_sensor, turn);
            turn_count = 0;
            last_turn = 0;
            continue;
        }

        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);


        //ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
#ifdef CONFIG_ENABLE_OLED
        // Displaying kp, ki, kd values on OLED 
        if (read_pid_const().val_changed)
        {
            esp_netif_ip_info_t ip_info;
            esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

            const char *ip_str_value = ip4addr_ntoa((const ip4_addr_t *)&ip_info.ip);
            display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, ip_str_value);

            reset_val_changed_pid_const();
        }
#endif

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    start_tuning_http_server();
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
}
