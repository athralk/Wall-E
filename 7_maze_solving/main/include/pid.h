#ifndef PID_H
#define PID_H

/*
 * PID controller for line following. Same maths as 5_line_following, moved
 * into its own file so the maze code can reset it after turns.
 * Kp / Ki / Kd are read live from the web tuner (tuning_http_server).
 */

/*
 * Returns the correction for the given line position error, using the
 * gains from the web tuner. The caller adds it to the left wheel and
 * subtracts it from the right wheel.
 */
float pid_compute(float error);

/*
 * Clears history, call after a turn so old error doesn't kick the bot
 */
void pid_reset(void);

/*
 * Last error passed to pid_compute() (x10). Its sign tells which side the
 * line was on just before it was lost, used to pick a search direction.
 */
float pid_last_error(void);

#endif
