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
    float position;           // -2 (line under left) .. +2 (line under right), in sensors, only valid if count > 0
    int raw_white_count;      // sensors seeing white, ignoring the inverted flag
    uint8_t raw_pattern;      // sensors seeing white as bits (like pattern), ignoring the inverted flag
    bool ir_left;             // legacy side-IR field; false when GPIO 27 is used as the front detector
    bool ir_object_detected;  // front-facing IR output matches IR_OBJECT_LEVEL
} lsa_state_t;

/*
 * Sets up the ADC channels for the LSA and the side IR pin, call once before line_sensor_update()
 */
esp_err_t line_sensor_init(void);

/*
 * Reads the LSA and fills in every field of state
 */
void line_sensor_update(lsa_state_t *state);

/*
 * Swaps which colour counts as the line, used by colour_blind.c for the
 * section where the track becomes a black line on white
 */
void line_sensor_set_inverted(bool inverted);

bool line_sensor_is_inverted(void);

/*
 * Copy of the latest reading, for the live LSA web page (any task may call it)
 */
lsa_state_t line_sensor_latest(void);

#endif
