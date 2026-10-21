/*
 * Left edge of the white line, see edge.h
 */
#include "edge.h"
#include "config.h"

int edge_first_white(const lsa_state_t *lsa)
{
    for (int i = 0; i < LSA_COUNT; i++)
    {
        if (lsa->on_line[i])
            return i;
    }
    return LSA_COUNT;
}

/*
 * Smooth edge position, in sensors from the left: how "black" each sensor is (0..1,
 * from the darkest to the brightest reading), added up from A0 to the first white one.
 * 00111 with A1 fully black = 2.0, A1 half white = 1.5. Moves smoothly as the bot drifts,
 * instead of jumping a whole sensor at a time (the jumps made the steering jitter).
 */
static float edge_position(const lsa_state_t *lsa)
{
    int first = edge_first_white(lsa);
    int lo = lsa->reading[0], hi = lsa->reading[0];
    for (int i = 1; i < LSA_COUNT; i++)
    {
        lo = lsa->reading[i] < lo ? lsa->reading[i] : lo;
        hi = lsa->reading[i] > hi ? lsa->reading[i] : hi;
    }
    if (first == LSA_COUNT || hi - lo < EDGE_MIN_CONTRAST)
        return first;       // all one colour: whole sensors only
    float pos = 0;
    for (int i = 0; i <= first; i++)
        pos += (float)(hi - lsa->reading[i]) / (hi - lo);
    return pos;
}

float edge_error(const lsa_state_t *lsa)
{
    // 0 = edge at A1|A2, negative = drifted right (steer left), positive = drifted left
    return (edge_position(lsa) - EDGE_TARGET) * EDGE_SCALE;
}

bool edge_left_branch(const lsa_state_t *lsa)
{
    return lsa->on_line[0] && lsa->on_line[1];
}

bool edge_on_target(const lsa_state_t *lsa)
{
    return edge_first_white(lsa) == EDGE_TARGET;
}
