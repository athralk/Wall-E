/*
 * PID controller, see pid.h
 *
 * Like the micromouse wall centering PD, the derivative uses the real time since
 * the last call (dt) instead of assuming every loop takes the same time. A slow
 * loop (e.g. the OLED redrawing) then doesn't make the D term spike. Everything
 * is scaled to a PID_DT_REF_S loop, so Kp / Ki / Kd keep the same meaning as
 * in 5_line_following.
 */
#include "pid.h"
#include "sra_board.h"
#include "esp_timer.h"
#include "config.h"
#include "tuning_http_server.h"

#define PID_DT_REF_S (LOOP_MS / 1000.0f)    // loop time the gains are tuned for
#define PID_DT_MAX_S 0.1f                   // longer gaps (after a turn / pause) count as one normal loop

static float prev_error = 0, cumulative_error = 0;
static int64_t last_us = 0;

float pid_compute(float error)
{
    // Gains come from the web page, so changes apply on the next loop
    pid_const_t k = read_pid_const();

    // Real time since the last call, in units of a normal loop
    int64_t now_us = esp_timer_get_time();
    float dt = last_us ? (now_us - last_us) / 1000000.0f : PID_DT_REF_S;
    last_us = now_us;
    if (!PID_REAL_DT || dt <= 0 || dt > PID_DT_MAX_S)
    {
        // PID_REAL_DT 0: every loop counts as one loop, same as 5_line_following
        dt = PID_DT_REF_S;
    }
    float loops = dt / PID_DT_REF_S;

    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle parameter
    float difference = (error - prev_error) / loops;                            // D: how fast the error is changing, per loop
    cumulative_error = bound(cumulative_error + error * loops, -30, 30);        // I: summed error, clamped so it can't wind up
    prev_error = error;

    return k.kp * error + k.ki * cumulative_error + k.kd * difference;
}

void pid_reset(void)
{
    prev_error = 0;
    cumulative_error = 0;
    last_us = 0;
}

float pid_last_error(void)
{
    return prev_error;
}
