#ifndef EDGE_H
#define EDGE_H

/*
 * What the sensor bar says about the left edge of the white line.
 * Normal following: black under A0 A1, white from A2 on (00111).
 */

#include <stdbool.h>
#include "line_sensor.h"

/*
 * Index of the leftmost white sensor, LSA_COUNT if there is no white
 */
int edge_first_white(const lsa_state_t *lsa);

/*
 * PID error: 0 = edge at its place, negative = edge moved left (steer left),
 * positive = edge moved right (steer right)
 */
float edge_error(const lsa_state_t *lsa);

/*
 * White under A0 and A1: a line coming in from the left (branch, corner, cross, T)
 */
bool edge_left_branch(const lsa_state_t *lsa);

/*
 * Edge back at its place: A0 A1 black, A2 white. Ends every turn.
 */
bool edge_on_target(const lsa_state_t *lsa);

#endif
