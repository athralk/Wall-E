#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#define LSA_COUNT 5

/*
 * Pattern bits, read left to right like the sensor bar:
 * bit 4 = A0 (leftmost) ... bit 0 = A4 (rightmost)
 */
#define LSA_BIT(i) (1 << (LSA_COUNT - 1 - (i)))

typedef struct
{
    int reading[LSA_COUNT];   // normalised 0-1000, higher = whiter
    bool on_line[LSA_COUNT];  // true when the sensor sees the (white) line
    uint8_t pattern;          // on_line packed as bits, see LSA_BIT
    int count;                // number of sensors on the line
    float position;           // -5 (line under left) .. +5 (line under right), only valid if count > 0
} lsa_state_t;

esp_err_t line_sensor_init(void);

/*
 * Reads the LSA and fills in every field of state
 */
void line_sensor_update(lsa_state_t *state);

/*
 * Swaps which colour counts as the line (hook for the colour blind section)
 */
void line_sensor_set_inverted(bool inverted);

#endif
