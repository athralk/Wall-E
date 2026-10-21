/*
 * Colour blind section, see colour_blind.h
 */
#include "sra_board.h"
#include "config.h"
#include "colour_blind.h"
#include "line_sensor.h"
#include "pid.h"

static const char *TAG = "colour blind";

static int enter_count = 0;     // loops of 10001 (white outside, black middle) in white line mode
static int leave_count = 0;     // loops of 01110 (white middle, black outside) in black line mode

/*
 * Raw patterns (1 = white, whatever mode we are in), bit 4 = leftmost sensor:
 *   enter black line mode: both outer sensors white, centre black  -> 10001 (also 11011 for a
 *                          narrow black line)
 *   back to white line:    both outer sensors black, centre white  -> 01110 (also 00100 for a
 *                          narrow white line)
 */
#define OUTER_BITS (LSA_BIT(0) | LSA_BIT(4))
#define CENTRE_BIT LSA_BIT(2)

bool colour_blind_update(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;
    bool inverted = line_sensor_is_inverted();
    uint8_t raw = lsa->raw_pattern;

    bool black_line = (raw & OUTER_BITS) == OUTER_BITS && !(raw & CENTRE_BIT);
    bool white_line = (raw & OUTER_BITS) == 0 && (raw & CENTRE_BIT);
    // Safety: in black line mode a bar with no white at all means we are off the block
    bool off_block = inverted && raw == 0;

    enter_count = !inverted && black_line ? enter_count + 1 : 0;
    leave_count = inverted && (white_line || off_block) ? leave_count + 1 : 0;

    // Never flip again during the hold after a flip: the block edges can show either pattern
    if (now_ms() < ctx->hold_until_ms && ctx->seen_line)
    {
        enter_count = leave_count = 0;
        return false;
    }

    if (enter_count < INVERT_CONFIRM && leave_count < INVERT_EXIT_CONFIRM)
    {
        return false;
    }
    enter_count = leave_count = 0;

    line_sensor_set_inverted(!inverted);
    line_sensor_update(&ctx->lsa);
    pid_reset();
    ctx->node_confirm = ctx->hits_left = ctx->hits_right = 0;
    line_width_reset(&ctx->line_width);     // the black line can be a different width
    ctx->sharp_dir = ctx->sharp_count = 0;
    ctx->lost_count = 0;
    ctx->last_centred = true;
    ctx->hold_until_ms = now_ms() + INVERT_HOLD_MS;
    ctx->display_dirty = true;

    ESP_LOGI(TAG, "%s (black line %s)", inverted ? "OFF" : "ON", inverted ? "ended" : "started");
    return true;
}
