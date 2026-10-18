/*
 * Colour blind section, see colour_blind.h
 */
#include "sra_board.h"
#include "config.h"
#include "colour_blind.h"
#include "line_sensor.h"
#include "pid.h"

static const char *TAG = "colour blind";

static int wrong_count = 0;     // loops the "polarity is wrong" pattern has held
static int exit_count = 0;      // loops of all white while inverted

bool colour_blind_update(maze_ctx_t *ctx)
{
    const lsa_state_t *lsa = &ctx->lsa;
    bool inverted = line_sensor_is_inverted();

    // Outer sensors on, centre off: we are looking at the gap around a line, not the line
    bool wrong = lsa->on_line[0] && lsa->on_line[4] && !lsa->on_line[2];
    wrong_count = wrong ? wrong_count + 1 : 0;

    // While inverted, a bar full of white is the end edge of the block, and a bar full of
    // black means we aren't on the block at all (left it, or flipped by mistake)
    bool exit_edge = inverted && (lsa->raw_white_count == LSA_COUNT || lsa->raw_white_count == 0);
    exit_count = exit_edge ? exit_count + 1 : 0;

    if (wrong_count < INVERT_CONFIRM && exit_count < INVERT_EXIT_CONFIRM)
    {
        return false;
    }
    wrong_count = exit_count = 0;

    line_sensor_set_inverted(!inverted);
    line_sensor_update(&ctx->lsa);
    pid_reset();
    ctx->node_seen = ctx->node_confirm = 0;
    ctx->sharp_dir = ctx->sharp_count = 0;
    ctx->lost_count = 0;
    ctx->last_centred = true;
    ctx->hold_until_ms = now_ms() + INVERT_HOLD_MS;
    ctx->display_dirty = true;

    ESP_LOGI(TAG, "%s (black line %s)", inverted ? "OFF" : "ON", inverted ? "ended" : "started");
    return true;
}
