#ifndef GYRO_H
#define GYRO_H

/*
 * Turn angle from the MPU-6050 gyro.
 *
 * The SRA library only turns the gyro into tilt (pitch / roll, for self-balancing), but
 * the chip measures rotation about all three axes. The z axis is the bot turning flat on
 * the mat, so adding up the z rate over time gives how far it has turned.
 */

#include <stdbool.h>

/*
 * Powers the MPU-6050 and measures the z rate while the bot stands still (the bias, the
 * small rate it reads when not turning). Call once at start-up with the motors off.
 * Returns false if the MPU doesn't answer (not plugged in): turns then use the arc fallback.
 */
bool gyro_init(void);

/*
 * True if gyro_init() found the MPU
 */
bool gyro_ok(void);

/*
 * Start measuring a turn from 0 degrees
 */
void gyro_reset(void);

/*
 * Reads the z rate and adds it to the angle. Call every loop while turning.
 * Returns how far the bot has turned since gyro_reset(), in degrees, always positive
 * (the direction is known from the turn, and this way it doesn't matter which way up
 * the chip is mounted).
 */
float gyro_turned(void);

#endif
