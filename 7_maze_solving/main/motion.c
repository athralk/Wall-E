#include "motion.h"
#include "sra_board.h"

static motor_handle_t motor_left, motor_right;

esp_err_t motion_init(void)
{
    ESP_ERROR_CHECK(enable_motor_driver(&motor_left, MOTOR_A_0));
    return enable_motor_driver(&motor_right, MOTOR_A_1);
}

static void set_wheel(motor_handle_t motor, float duty)
{
    duty = bound(duty, -50, 50);
    if (duty >= 0)
    {
        set_motor_speed(motor, MOTOR_FORWARD, duty);
    }
    else
    {
        set_motor_speed(motor, MOTOR_BACKWARD, -duty);
    }
}

void drive(float left_duty, float right_duty)
{
    set_wheel(motor_left, left_duty);
    set_wheel(motor_right, right_duty);
}

void pivot(int dir, float duty)
{
    drive(dir * duty, -dir * duty);
}

void stop(void)
{
    set_motor_speed(motor_left, MOTOR_STOP, 0);
    set_motor_speed(motor_right, MOTOR_STOP, 0);
}
