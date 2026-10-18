/*
 * Line sensor array: raw ADC readings -> on_line flags, bit pattern and position.
 * See line_sensor.h for what each field of lsa_state_t means.
 */
#include "line_sensor.h"
#include "sra_board.h"

/*
 * Raw ADC range (0-4095) is clamped to these margins, then mapped to 0-1000
 */
#define BLACK_MARGIN 4095 //TESTED VALUE CAN CHANGE <<<<<<<<<=========================
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define WHITE_BOUNDARY 820 // normalised readings above this are on the white line

/*
 * Symmetric weights, scaled by POSITION_SCALE so position spans -5..+5
 * like the error range the original line_following example was tuned with
 */
static const int weights[LSA_COUNT] = {-2, -1, 0, 1, 2};
#define POSITION_SCALE 2.5f

static adc_handle_t line_sensor;
static bool inverted = false;   // true = black counts as the line (colour blind section)

esp_err_t line_sensor_init(void)
{
    return enable_line_sensor(&line_sensor);
}

void line_sensor_set_inverted(bool value)
{
    inverted = value;
}

void line_sensor_update(lsa_state_t *state)
{
    line_sensor_array raw = read_line_sensor(line_sensor);
    float weighted_sum = 0;

    state->pattern = 0;
    state->count = 0;

    for (int i = 0; i < LSA_COUNT; i++)
    {
        // Clamp and scale the raw reading to 0-1000, then flip it so white reads high
        int value = bound(raw.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
        value = map(value, WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        value = 1000 - value;
        state->reading[i] = value;

        // Threshold into on / off the line
        bool on = value > WHITE_BOUNDARY;
        if (inverted)
        {
            on = !on;
        }
        state->on_line[i] = on;

        // Build the bit pattern, the count and the sum for the position
        if (on)
        {
            state->pattern |= LSA_BIT(i);
            state->count++;
            weighted_sum += weights[i];
        }
    }

    // Average weight of the sensors on the line: 0 = centred, negative = line is to the left
    state->position = state->count ? (weighted_sum / state->count) * POSITION_SCALE : 0;
}
