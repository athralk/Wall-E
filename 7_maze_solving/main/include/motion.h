#ifndef MOTION_H
#define MOTION_H

#include "esp_err.h"

esp_err_t motion_init(void);

/*
 * Signed duty cycles (-100..100), negative drives that wheel backward
 */
void drive(float left_duty, float right_duty);

/*
 * Spins in place, dir = -1 for left, +1 for right
 */
void pivot(int dir, float duty);

void stop(void);

#endif
