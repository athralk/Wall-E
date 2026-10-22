#ifndef EDGE_H
#define EDGE_H

/*
 * What the sensor bar says about the line: its centre (centre mode) or its left edge
 * (edge mode: black under A0 A1, white from A2 on, 00111).
 */

#include <stdbool.h>
#include "line_sensor.h"

/*
 * Index of the leftmost white sensor, LSA_COUNT if there is no white
 */
int edge_first_white(const lsa_state_t *lsa);

/*
 * PID errors, -5..+5: negative = line / edge is left of its place (steer left),
 * positive = right of it (steer right). Both are smooth (from the analog readings),
 * so the steering doesn't jump a whole sensor at a time.
 */
float centre_error(const lsa_state_t *lsa);
float edge_error(const lsa_state_t *lsa);

/*
 * Edge mode: white under A0 and A1, a line coming in from the left
 */
bool edge_left_branch(const lsa_state_t *lsa);

#endif
