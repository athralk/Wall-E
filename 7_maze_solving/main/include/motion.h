#ifndef MOTION_H
#define MOTION_H

/*
 * Motor wrapper. The SRA library only takes a direction plus a positive duty,
 * this turns that into signed duties so one call can drive a wheel forward or
 * backward, which is what lets the bot pivot in place on its ball caster.
 *
 * Left wheel = MOTOR_A_0, right wheel = MOTOR_A_1.
 */

#include "esp_err.h"

/*
 * Enables both motor channels, call once at startup
 */
esp_err_t motion_init(void);

/*
 * Signed duty cycles, negative drives that wheel backward.
 * Each wheel is capped at +-50 in motion.c, so higher values act as 50.
 */
void drive(float left_duty, float right_duty);

/*
 * Spins in place, dir = -1 for left, +1 for right
 * (one wheel forward, the other backward at the same duty)
 */
void pivot(int dir, float duty);

/*
 * Stops both wheels
 */
void stop(void);

#endif
