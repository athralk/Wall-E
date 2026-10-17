#include "pid.h"
#include "sra_board.h"
#include "tuning_http_server.h"

static float prev_error = 0, cumulative_error = 0;

float pid_compute(float error)
{
    pid_const_t k = read_pid_const();

    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle parameter
    float difference = error - prev_error;
    cumulative_error = bound(cumulative_error + error, -30, 30);
    prev_error = error;

    return k.kp * error + k.ki * cumulative_error + k.kd * difference;
}

void pid_reset(void)
{
    prev_error = 0;
    cumulative_error = 0;
}

float pid_last_error(void)
{
    return prev_error;
}
