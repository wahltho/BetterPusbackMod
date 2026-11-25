#ifndef BP_PUSHBACK_MOTION_FORCE_H
#define BP_PUSHBACK_MOTION_FORCE_H

#include <math.h>
#include <stdbool.h>

/* Fast master motion only. Forces are signed newtons, integral is m/s^2. */
typedef struct {
    double integral, force, limit, requested_force;
} bp_motion_force_t;

static inline bool
bp_motion_sample_ok(double factor, double dt)
{
    return ((factor == 1.5 || factor == 2) && dt > 0 && dt <= 0.1);
}

static inline double
bp_motion_clamp(double v, double lo, double hi)
{
    return (fmin(fmax(v, lo), hi));
}

static inline bool
bp_motion_pi(double factor, double dt, double mass, double force_per_ton,
    double max_accel, double target, double speed, double integral,
    double previous, double brake, bp_motion_force_t *out)
{
    double limit = (mass / 1000) * force_per_ton;
    double step = limit * dt, a = factor * factor * max_accel;
    double f, requested;

    *out = (bp_motion_force_t){0};
    if (!bp_motion_sample_ok(factor, dt) || !isfinite(mass) || mass <= 0 ||
        !isfinite(force_per_ton) || force_per_ton <= 0 ||
        !isfinite(limit) || limit <= 0 || !isfinite(a) || a <= 0 ||
        !isfinite(target) || !isfinite(speed) || !isfinite(integral) ||
        !isfinite(previous) || isinf(brake))
        return (false);

    if (isnan(brake)) {
        double e = target - speed, p = (2 * factor) * e;
        double next_i = integral + (factor * factor * dt) * e;
        double demand = p + next_i;
        /* All three saturations, expressed in acceleration units. Do not
         * freeze an increment that takes us back out of saturation. */
        double hi = fmin(a, fmin(limit, previous + step) / mass);
        double lo = fmax(-a, fmax(-limit, previous - step) / mass);

        if (!isfinite(next_i) || !isfinite(demand))
            return (false);
        if (!((e > 0 && demand > hi) || (e < 0 && demand < lo)))
            integral = next_i;
        if (!isfinite(p + integral))
            return (false);
        requested = mass * (p + integral);
        f = mass * bp_motion_clamp(p + integral, -a, a);
    } else {
        integral = 0;
        requested = f = brake;
    }
    if (!isfinite(f) || !isfinite(requested))
        return (false);
    f = bp_motion_clamp(f, previous - step, previous + step);
    *out = (bp_motion_force_t){integral, bp_motion_clamp(f, -limit, limit),
        limit, requested};
    return (true);
}

/* G4 decays the last applied force, not the PI or envelope candidate. */
static inline void
bp_motion_no_contact(bp_motion_force_t *cmd, double previous, double dt)
{
    double residual = fmax(0, fabs(previous) - cmd->limit * (2 * dt));
    cmd->integral = 0;
    cmd->force = copysign(fmin(residual, cmd->limit), previous);
}

static inline double
bp_motion_turn_increment(double speed_ref, double circumference,
    double yaw_delta, double dt, double factor)
{
    double rate_ref = (speed_ref / circumference) * 360 -
        (yaw_delta / dt) / factor;
    return (rate_ref * (dt * factor));
}

/* Preserve the original signed comparison, including its reverse asymmetry. */
static inline double
bp_motion_limited_steer(double steer, double speed_ref, double limited)
{
    return (limited < speed_ref ? steer * (limited / speed_ref) : steer);
}

#endif
