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
 * Symmetric weights, scaled by POSITION_SCALE so position spans -5..+5
 * like the error range the original line_following example was tuned with
 */
static const int weights[LSA_COUNT] = {-2, -1, 0, 1, 2};
#define POSITION_SCALE 2.5f

static adc_handle_t line_sensor;
static lsa_state_t latest;      // last reading, for the live LSA web page
static bool inverted = false;   // true = black counts as the line (colour blind section)
static volatile bool ir_latched = false;    // side IR went onto white since the last update

#if IR_ENABLED
static void IRAM_ATTR ir_isr(void *arg)
{
    ir_latched = true;
}
#endif

esp_err_t line_sensor_init(void)
{
#if IR_ENABLED
    // Side IR: plain digital input, the module drives OUT itself
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << IR_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,       // on top of the module's own, for cleaner edges
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        // Edge onto white, so a short pass between two loops is remembered
        .intr_type = IR_WHITE_LEVEL ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        return err;
    }
    // The ISR service may already be installed by another driver, that's fine
    // IRAM + level 3: the ISR runs even during flash writes and isn't delayed by Wi-Fi interrupts
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }
    err = gpio_isr_handler_add(IR_PIN, ir_isr, NULL);
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

#if IR_ENABLED
    // Side IR: same white / black rule as the LSA, flipped in the colour blind section.
    // Normal track: also counts a white pass caught by the interrupt since the last loop.
    bool ir_white = gpio_get_level(IR_PIN) == IR_WHITE_LEVEL;
    bool latched = ir_latched;
    ir_latched = false;
    state->ir_left = inverted ? !ir_white : (ir_white || latched);
#else
    state->ir_left = false;
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
