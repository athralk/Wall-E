/*
 * Line centre / left edge from the sensor bar, see edge.h
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
 * Darkest and brightest reading under the bar. False if they are too close to tell
 * the line from the floor (all one colour): then only the on / off flags are used.
 */
static bool contrast(const lsa_state_t *lsa, int *lo, int *hi)
{
    *lo = *hi = lsa->reading[0];
    for (int i = 1; i < LSA_COUNT; i++)
    {
        *lo = lsa->reading[i] < *lo ? lsa->reading[i] : *lo;
        *hi = lsa->reading[i] > *hi ? lsa->reading[i] : *hi;
    }
    return *hi - *lo >= EDGE_MIN_CONTRAST;
}

/*
 * Centre: every sensor pulls toward its own place by how white it is (0..1 between the
 * darkest and brightest reading), so the position slides smoothly between sensors
 */
float centre_error(const lsa_state_t *lsa)
{
    int lo, hi;
    if (!contrast(lsa, &lo, &hi))
        return lsa->position;       // on / off flags only
    float sum = 0, total = 0;
    for (int i = 0; i < LSA_COUNT; i++)
    {
        float w = (float)(lsa->reading[i] - lo) / (hi - lo);
        sum += w * (i - LSA_COUNT / 2);
        total += w;
    }
    return total > 0 ? sum / total * EDGE_SCALE : 0;
}

/*
 * Edge: how black each sensor is, added up from A0 to the first white one.
 * 00111 with A1 fully black = 2.0 (in place), A1 half white = 1.5.
 */
float edge_error(const lsa_state_t *lsa)
{
    int first = edge_first_white(lsa);
    int lo, hi;
    float pos = first;
    if (first < LSA_COUNT && contrast(lsa, &lo, &hi))
    {
        pos = 0;
        for (int i = 0; i <= first; i++)
            pos += (float)(hi - lsa->reading[i]) / (hi - lo);
    }
    return (pos - EDGE_TARGET) * EDGE_SCALE;
}

bool edge_left_branch(const lsa_state_t *lsa)
{
    return lsa->on_line[0] && lsa->on_line[1];
}
