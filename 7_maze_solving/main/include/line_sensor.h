#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

/*
 * Line sensor array (LSA) wrapper.
 *
 * Reads the 5 analog sensors once per loop and turns them into everything the
 * maze logic needs: which sensors see the line, a bit pattern for spotting
 * corners / junctions, and a single position number for the PID.
 *
 * Sensor index 0 is the leftmost sensor (A0), index 4 the rightmost (A4).
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LSA_COUNT 5

/*
 * Pattern bits, read left to right like the sensor bar:
 * bit 4 = A0 (leftmost) ... bit 0 = A4 (rightmost)
 * so pattern 0b11100 means the three left sensors are on the line
 */
#define LSA_BIT(i) (1 << (LSA_COUNT - 1 - (i)))

/*
 * One snapshot of the sensor bar, refreshed by line_sensor_update() every loop
 */
typedef struct
{
    int reading[LSA_COUNT];   // normalised 0-1000, higher = whiter
    bool on_line[LSA_COUNT];  // true when the sensor sees the (white) line
    uint8_t pattern;          // on_line packed as bits, see LSA_BIT
    int count;                // number of sensors on the line (0 = line lost, 5 = all white)
    float position;           // -5 (line under left) .. +5 (line under right), only valid if count > 0
} lsa_state_t;

/*
 * Sets up the ADC channels for the LSA, call once before line_sensor_update()
 */
esp_err_t line_sensor_init(void);

/*
 * Reads the LSA and fills in every field of state
 */
void line_sensor_update(lsa_state_t *state);

/*
 * Swaps which colour counts as the line (hook for the colour blind section,
 * where the track becomes a black line on white). Not used yet.
 */
void line_sensor_set_inverted(bool inverted);

#endif
