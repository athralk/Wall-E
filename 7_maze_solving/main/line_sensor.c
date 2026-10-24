/*
 * Line sensor array: raw ADC readings -> on_line flags, bit pattern and position.
 * See line_sensor.h for what each field of lsa_state_t means.
 */
#include "line_sensor.h"
#include "sra_board.h"
#include "driver/gpio.h"
#include "config.h"

/*
 * Raw ADC range (0-4095) is clamped to these margins, then mapped to 0-1000
 */
#define BLACK_MARGIN 4095 //TESTED VALUE CAN CHANGE <<<<<<<<<=========================
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define WHITE_BOUNDARY 820 // normalised readings above this are on the white line

/*
 * Symmetric weights: position is in sensors, -2 (line under A0) .. +2 (under A4)
 */
static const int weights[LSA_COUNT] = {-2, -1, 0, 1, 2};
#define POSITION_SCALE 1.0f

static adc_handle_t line_sensor;
static lsa_state_t latest;      // last reading, for the live LSA web page
static bool inverted = false;   // true = black counts as the line (colour blind section)

esp_err_t line_sensor_init(void)
{
#if IR_ENABLED
    // Front-facing IR detector: sample the module's digital OUT directly, as in main.c demo.
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        return err;
    }
#endif
    return enable_line_sensor(&line_sensor);
}

void line_sensor_set_inverted(bool value)
{
    inverted = value;
}

bool line_sensor_is_inverted(void)
{
    return inverted;
}

/*
 * Line position from the analog readings, -5 .. +5 like the on / off position.
 *
 * Each sensor's "line strength" is how far its reading is above the darkest sensor
 * (below the brightest one in the colour blind section, where the line is black).
 * Using the darkest / brightest sensor as the reference means no black or white
 * calibration is needed. The position is the strength-weighted average of the
 * sensor weights, so a line halfway between two sensors gives a position halfway
 * between them.
 *
 * Falls back to the on / off position if the readings are too flat to trust
 * (all sensors within MIN_CONTRAST of each other).
 */
__attribute__((unused)) static float analog_position(const int reading[LSA_COUNT])
{
    int lowest = reading[0], highest = reading[0];
    for (int i = 1; i < LSA_COUNT; i++)
    {
        if (reading[i] < lowest)
            lowest = reading[i];
        if (reading[i] > highest)
            highest = reading[i];
    }

    float weighted = 0, total = 0;
    int range = highest - lowest;
    if (range >= MIN_CONTRAST)
    {
        for (int i = 0; i < LSA_COUNT; i++)
        {
            float strength = inverted ? highest - reading[i] : reading[i] - lowest;
            // Ignore the bottom 20% of the range: sensors that only see background noise
            // would otherwise drag the position toward the middle
            strength -= 0.2f * range;
            if (strength < 0)
            {
                strength = 0;
            }
            weighted += weights[i] * strength;
            total += strength;
        }
    }
    if (total <= 0)
    {
        return 0;
    }
    return (weighted / total) * POSITION_SCALE;
}

void line_sensor_update(lsa_state_t *state)
{
    line_sensor_array raw = read_line_sensor(line_sensor);
    float weighted_sum = 0;

    state->pattern = 0;
    state->count = 0;
    state->raw_white_count = 0;
    state->raw_pattern = 0;

    for (int i = 0; i < LSA_COUNT; i++)
    {
        // Clamp and scale the raw reading to 0-1000, then flip it so white reads high
        int value = bound(raw.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
        value = map(value, WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        value = 1000 - value;
        state->reading[i] = value;

        // Threshold into on / off the line
        bool on = value > WHITE_BOUNDARY;
        if (on)
        {
            state->raw_white_count++;
            state->raw_pattern |= LSA_BIT(i);
        }
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

    // GPIO 27 is a front-facing object detector now, not a side-branch sensor.
    // Read its current digital level; the LSA white condition is checked in FOLLOW.
    state->ir_left = false;
#if IR_ENABLED
    state->ir_object_detected = (gpio_get_level(IR_PIN) == IR_OBJECT_LEVEL);
#else
    state->ir_object_detected = false;
#endif

    // Coarse position from the on / off flags: average weight of the sensors on the line,
    // 0 = centred, negative = line is to the left. Only a few possible values.
    state->position = state->count ? (weighted_sum / state->count) * POSITION_SCALE : 0;
    latest = *state;

#if ANALOG_CENTERING
    // Fine position from the analog readings, like wall centering on the micromouse
    // (error = right reading - left reading): every sensor pulls toward its side by how
    // strongly it sees the line, so the position changes smoothly as the bot drifts.
    if (state->count > 0)
    {
        state->position = analog_position(state->reading);
    }
#endif
}

lsa_state_t line_sensor_latest(void)
{
    return latest;
}
