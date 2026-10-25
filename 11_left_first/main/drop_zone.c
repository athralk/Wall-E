/*
 * Pink drop zone, see drop_zone.h
 *
 * DROP state timeline:
 *   0 .. DROP_STOP_MS                      stand still
 *   .. DROP_STOP_MS + DROP_REVERSE_MS      reverse off the object
 *   then                                   U-turn (UTURN, uturn.c), then the maze as normal
 */
#include "sra_board.h"
#include "config.h"
#include "drop_zone.h"
#include "motion.h"

static const char *TAG = "drop zone";

static int pink_count = 0;

static bool is_pink(int reading)
{
    return reading >= PINK_MIN && reading <= PINK_MAX;
}

drop_check_t drop_zone_check(maze_ctx_t *ctx)
{
    if (!DROP_ZONE_ENABLED || ctx->drop_done)
    {
        return DROP_NONE;
    }

    // reading[] is before any colour blind flip, so this works in both modes
    const int *r = ctx->lsa.reading;
    if (!(is_pink(r[1]) && is_pink(r[2]) && is_pink(r[3])))
    {
        pink_count = 0;
        return DROP_NONE;
    }

    return ++pink_count >= PINK_CONFIRM ? DROP_CONFIRMED : DROP_MAYBE;
}

void start_drop(maze_ctx_t *ctx)
{
    pink_count = 0;
    ctx->drop_done = true;
    ESP_LOGI(TAG, "pink reached  readings %d %d %d", ctx->lsa.reading[1], ctx->lsa.reading[2], ctx->lsa.reading[3]);
    stop();
    set_state(ctx, STATE_DROP);
}

void handle_drop(maze_ctx_t *ctx)
{
    uint32_t elapsed = state_elapsed(ctx);

    if (elapsed < DROP_STOP_MS)
    {
        stop();
    }
    else if (elapsed < DROP_STOP_MS + DROP_REVERSE_MS)
    {
        drive(-DROP_REVERSE_DUTY, -DROP_REVERSE_DUTY);
    }
    else
    {
        ESP_LOGI(TAG, "object dropped, U-turn");
        start_uturn(ctx);
    }
}
