/* SPDX-License-Identifier: CDDL-1.0 */
#ifndef BP_MOTION_ENVELOPE_H
#define BP_MOTION_ENVELOPE_H

#include <math.h>
#include <stdbool.h>

/* Physical aircraft-longitudinal SI units, except force (signed at the
 * tug), headings/steer (degrees clockwise from north), and direction (+/-1).
 * This header has no simulator, list, or libacfutils dependency. */
#define MOTION_ENV_RAD (0.01745329251994329577)
#define MOTION_ENV_HORIZON_S 0.1

typedef enum {
    MOTION_BOUND_NONE, MOTION_BOUND_CURVE, MOTION_BOUND_END, MOTION_BOUND_REVERSE
} motion_boundary_t;

typedef struct {
    int direction;           /* segment traversal, even during rollback */
    double actual_mps;       /* magnitude of measured longitudinal speed */
    double requested_mps;    /* physical cruise ceiling, not a PI target */
    double allowed_mps;      /* absolute longitudinal ceiling */
    double distance_m, entry_mps; /* active restriction, when binding */
    double b_mps2, reaction_s, accel_plus_mps2;
    double brake_force_n;    /* signed force target; apply only if intervene */
    motion_boundary_t boundary;
    bool binding, intervene;
} motion_envelope_t;

/* Initialize one instantaneous, read-only query. No F=1 execution path.
 * Use measured longitudinal speed and the LAST physically delivered force,
 * not measured tug speed, a requested force, or a planned steering angle.
 * At standstill anticipate the route direction. During rollback oppose the
 * measured motion, never accelerate it using the intended route direction. */
static inline bool
motion_envelope_begin(motion_envelope_t *out, int direction, double factor,
    double actual_mps, double requested_mps, double steer_deg, double mass_kg,
    double force_cap_n, double last_force_n)
{
    motion_envelope_t e = {0};
    if ((direction != -1 && direction != 1) ||
        (factor != 1.5 && factor != 2.0) || !isfinite(actual_mps) ||
        !isfinite(requested_mps) || requested_mps < 0 ||
        !isfinite(steer_deg) || fabs(steer_deg) >= 90 ||
        !isfinite(mass_kg) || mass_kg <= 0 ||
        !isfinite(force_cap_n) || force_cap_n <= 0 || !isfinite(last_force_n))
        return false;

    const double g = cos(steer_deg * MOTION_ENV_RAD);
    const double authority = 0.25 * factor * factor;
    const double b = fmin(0.17 * factor * factor,
        0.8 * g * fmin(authority, force_cap_n / mass_kg));
    if (!(g > 0 && b > 0))
        return false;
    const double brake = mass_kg * (b / g);
    const int motion_sign = actual_mps == 0 ? direction :
        (actual_mps > 0 ? 1 : -1);
    const double p_hi = fmin(force_cap_n, motion_sign * last_force_n +
        force_cap_n * MOTION_ENV_HORIZON_S);
    e.reaction_s = MOTION_ENV_HORIZON_S +
        fmax(0, p_hi + brake) / force_cap_n;
    e.accel_plus_mps2 = fmax(authority, p_hi / mass_kg);
    if (!isfinite(brake) || brake <= 0 || brake > force_cap_n ||
        !isfinite(e.reaction_s) || !isfinite(e.accel_plus_mps2))
        return false;
    e.direction = direction;
    e.actual_mps = fabs(actual_mps);
    e.requested_mps = e.allowed_mps = requested_mps;
    e.b_mps2 = b;
    e.brake_force_n = -motion_sign * brake;
    *out = e;
    return true;
}

/* Called only for v > w. Overflow means insufficient distance, not an
 * unrestricted route. Factoring the difference of squares avoids inf-inf. */
static inline double
motion_envelope_distance(const motion_envelope_t *e, double v, double w)
{
    double t = e->reaction_s, a = e->accel_plus_mps2;
    double vr = v + a * t;
    return v * t + 0.5 * a * t * t +
        (vr - w) * (vr + w) / (2 * e->b_mps2);
}

/* Intersect one boundary with all preceding bounds. At w there is no
 * braking demand. A tie goes to the lower entry speed so a simultaneous
 * downstream restriction is not masked by the current curve ceiling. */
static inline bool
motion_envelope_bound(motion_envelope_t *e, double d, double w,
    motion_boundary_t boundary)
{
    if (!isfinite(d) || d < 0 || !isfinite(w) || w < 0)
        return false;
    double lo = w, hi = e->requested_mps;
    if (hi <= w || motion_envelope_distance(e, hi, w) <= d)
        return true;
    for (int i = 0; i < 24; i++) {
        double mid = lo + (hi - lo) * 0.5;
        if (motion_envelope_distance(e, mid, w) <= d)
            lo = mid;
        else
            hi = mid;
    }
    if (!e->binding || lo < e->allowed_mps ||
        (lo == e->allowed_mps && w < e->entry_mps)) {
        e->binding = true;
        e->allowed_mps = lo;
        e->entry_mps = w;
        e->distance_m = d;
        e->boundary = boundary;
        e->intervene = e->actual_mps >= lo && e->actual_mps > w;
    }
    return true;
}

static inline double
motion_envelope_sweep(double from, double to, int sense)
{
    double arc = fmod(sense * (to - from), 360.0);
    return arc < 0 ? arc + 360 : arc;
}

/* The turn center is on the physical right/left of the starting heading.
 * Reversing changes the traversal sense, not the center. If off the arc,
 * project to the nearest angular endpoint rather than adding a revolution. */
static inline double
motion_envelope_arc_m(double sx, double sy, double start_deg, double end_deg,
    double radius, bool right, bool backward, bool current, double x, double y)
{
    int side = right ? 1 : -1, sense = backward ? -side : side;
    double arc = motion_envelope_sweep(start_deg, end_deg, sense);
    if (current) {
        double dx = x - (sx + side * radius * cos(start_deg * MOTION_ENV_RAD));
        double dy = y - (sy - side * radius * sin(start_deg * MOTION_ENV_RAD));
        if (!isfinite(dx) || !isfinite(dy) || (dx == 0 && dy == 0))
            return NAN;
        double radial = atan2(dx, dy) / MOTION_ENV_RAD;
        double start_radial = start_deg - side * 90;
        double along = motion_envelope_sweep(start_radial, radial, sense);
        if (along <= arc)
            arc -= along;
        else if (along - arc <= 360 - along)
            arc = 0;
        /* Otherwise we are before the start; use the full arc. */
    }
    return radius * (arc * MOTION_ENV_RAD);
}

/* Distance is accumulated in traversal order. A curve constrains its
 * entrance, BEFORE its arc length is added, including a curve occupied
 * now (entrance distance zero). No heading error substitutes for arc length. */
static inline bool
motion_envelope_curve(motion_envelope_t *e, double *distance, double factor,
    double angular_deg_s, double lateral_mps2, double sx, double sy,
    double start_deg, double end_deg, double radius, bool right, bool current,
    double x, double y)
{
    if (!isfinite(radius) || radius <= 0 || !isfinite(angular_deg_s) ||
        angular_deg_s <= 0 || !isfinite(lateral_mps2) || lateral_mps2 <= 0 ||
        !isfinite(sx) || !isfinite(sy) || !isfinite(start_deg) ||
        !isfinite(end_deg))
        return false;
    double arc = motion_envelope_arc_m(sx, sy, start_deg, end_deg, radius,
        right, e->direction < 0, current, x, y);
    double w = factor * fmin((angular_deg_s * MOTION_ENV_RAD) * radius,
        sqrt(lateral_mps2) * sqrt(radius));
    if (!isfinite(arc) || !motion_envelope_bound(e, *distance, w,
        MOTION_BOUND_CURVE))
        return false;
    *distance += arc;
    return isfinite(*distance);
}

static inline bool
motion_envelope_straight(double *distance, int direction, double length,
    double sx, double sy, double heading, bool current, double x, double y)
{
    if (!isfinite(length) || length < 0 || !isfinite(sx) || !isfinite(sy) ||
        !isfinite(heading))
        return false;
    if (current) {
        double rad = heading * MOTION_ENV_RAD;
        double along = direction * ((x - sx) * sin(rad) + (y - sy) * cos(rad));
        if (!isfinite(along))
            return false;
        length = fmax(0, length - along);
    }
    *distance += length;
    return isfinite(*distance);
}

typedef enum {
    MOTION_ENV_OK, MOTION_ENV_INVALID, MOTION_ENV_OVERSPEED,
    MOTION_ENV_NO_DECEL
} motion_envelope_fault_t;

typedef struct {
    int direction, overspeed_samples;
    bool was_intervening;
    double previous_mps, no_decel_s, previous_brake_n;
} motion_envelope_watch_t;

/* Call exactly once per valid real motion sample, before releasing the head.
 * last_force_n was applied over the just-observed interval. Compare with the
 * signed target from its START, not a new target from changed steering.
 * Zero the watch on hold, discontinuity or re-entry. Caller latches faults. */
static inline motion_envelope_fault_t
motion_envelope_monitor(motion_envelope_watch_t *watch,
    const motion_envelope_t *e, double last_force_n, double dt)
{
    if (!isfinite(dt) || dt <= 0 || dt > MOTION_ENV_HORIZON_S ||
        !isfinite(last_force_n))
        return MOTION_ENV_INVALID;
    if (watch->direction != e->direction) {
        motion_envelope_watch_t fresh = {0};
        *watch = fresh;
        watch->direction = e->direction;
    }
    if (e->actual_mps - e->allowed_mps > 0.15) {
        if (watch->overspeed_samples < 2)
            watch->overspeed_samples++;
    } else {
        watch->overspeed_samples = 0;
    }
    if (watch->was_intervening && e->intervene &&
        (e->brake_force_n > 0) == (watch->previous_brake_n > 0) &&
        (watch->previous_brake_n > 0 ? last_force_n : -last_force_n) >=
        fabs(watch->previous_brake_n) && e->actual_mps >= watch->previous_mps)
        watch->no_decel_s += dt;
    else
        watch->no_decel_s = 0;
    watch->previous_mps = e->actual_mps;
    watch->previous_brake_n = e->brake_force_n;
    watch->was_intervening = e->intervene;
    if (watch->overspeed_samples >= 2)
        return MOTION_ENV_OVERSPEED;
    if (watch->no_decel_s >= 0.25 - 1e-12)
        return MOTION_ENV_NO_DECEL;
    return MOTION_ENV_OK;
}

#endif /* BP_MOTION_ENVELOPE_H */
