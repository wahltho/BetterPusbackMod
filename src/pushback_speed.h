/*
 * CDDL HEADER START
 * This file is supplied under the Common Development and Distribution
 * License (CDDL), version 1.0. See COPYING for the full license text.
 * CDDL HEADER END
 */
#ifndef BP_PUSHBACK_SPEED_H
#define BP_PUSHBACK_SPEED_H

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    double fwd_mps;
    double rev_mps;
} drive_speed_limits_t;

static inline double
bp_pushback_speed_normalize(double factor)
{
    return (factor == 1.5 || factor == 2.0 ? factor : 1.0);
}

static inline double
bp_pushback_speed_choice(int selection)
{
    return selection == 1 ? 1.5 : (selection == 2 ? 2.0 : 1.0);
}

static inline bool
bp_pushback_speed_profile(double factor, double legacy_fwd, double legacy_rev,
    double tug_fwd, double tug_rev, drive_speed_limits_t *limits)
{
    if (limits == NULL)
        return false;
    limits->fwd_mps = limits->rev_mps = 0;
    if (bp_pushback_speed_normalize(factor) == 1.0 ||
        !isfinite(legacy_fwd) || legacy_fwd <= 0 ||
        !isfinite(legacy_rev) || legacy_rev <= 0 ||
        !isfinite(tug_fwd) || tug_fwd <= 0 ||
        !isfinite(tug_rev) || tug_rev <= 0)
        return false;
    /* Bound overflow before multiplying, then cap the rounded product. */
    limits->fwd_mps = legacy_fwd > DBL_MAX / factor ? tug_fwd :
        fmin(legacy_fwd * factor, tug_fwd);
    limits->rev_mps = legacy_rev > DBL_MAX / factor ? tug_rev :
        fmin(legacy_rev * factor, tug_rev);
    return true;
}

static inline double
bp_pushback_speed_tug_cap(double speed, double tug_fwd, double tug_rev)
{
    if (!isfinite(speed) || !isfinite(tug_fwd) || tug_fwd <= 0 ||
        !isfinite(tug_rev) || tug_rev <= 0)
        return 0;
    if (speed > 0)
        return fmin(speed, tug_fwd);
    if (speed < 0)
        return fmax(speed, -tug_rev);
    return speed;
}

#endif /* BP_PUSHBACK_SPEED_H */
