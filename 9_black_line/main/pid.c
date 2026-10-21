/*
 * PID controller, see pid.h
 */
#include "pid.h"
#include "sra_board.h"
#include "tuning_http_server.h"

static float prev_error = 0, cumulative_error = 0;
static float smooth_difference = 0;     // D after a light low pass, so one noisy reading can't jerk

float pid_compute(float error)
{
    // Gains come from the web page, so changes apply on the next loop
    pid_const_t k = read_pid_const();

    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle parameter
    float difference = error - prev_error;                               // D: how fast the error is changing
    cumulative_error = bound(cumulative_error + error, -30, 30);         // I: summed error, clamped so it can't wind up
    prev_error = error;
    smooth_difference += 0.5f * (difference - smooth_difference);

    return k.kp * error + k.ki * cumulative_error + k.kd * smooth_difference;
}

void pid_reset(void)
{
    prev_error = 0;
    cumulative_error = 0;
    smooth_difference = 0;
}

float pid_last_error(void)
{
    return prev_error;
}
