#ifndef PID_H
#define PID_H

/*
 * Returns the correction for the given line position error, using the
 * gains from the web tuner
 */
float pid_compute(float error);

/*
 * Clears history, call after a turn so old error doesn't kick the bot
 */
void pid_reset(void);

float pid_last_error(void);

#endif
