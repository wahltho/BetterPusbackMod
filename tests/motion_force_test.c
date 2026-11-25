#include <assert.h>
#include <float.h>
#include <math.h>
#include <stddef.h>

#include "../src/pushback_motion_force.h"

static void
near(double actual, double expected)
{
    assert(isfinite(actual) && fabs(actual - expected) < 1e-9);
}

static bp_motion_force_t
step(double e, double a, double i, double prev, double brake)
{
    bp_motion_force_t cmd;
    assert(bp_motion_pi(2, 0.05, 1000, 5000, a, e, 0, i, prev,
        brake, &cmd));
    return cmd;
}

static void
limits_and_contact(void)
{
    for (int s = -1; s <= 1; s += 2) {
        bp_motion_force_t cmd = step(s, 0.25, 0, 0, NAN);
        near(cmd.limit, 5000);
        near(cmd.requested_force, s * 4000);
        near(cmd.force, s * 250);              /* slew, not a breakaway boost */
        near(cmd.integral, 0);
        cmd = step(s, 0.25, 0, s * 1000, NAN);
        near(cmd.force, s * 1000);             /* acceleration saturation */
        near(cmd.integral, 0);
        cmd = step(s * 10, 10, 0, s * 5000, NAN);
        near(cmd.force, s * 5000);             /* absolute force saturation */
        near(cmd.integral, 0);
        cmd = step(s * 0.01, 1, 0, 0, NAN);
        near(cmd.integral, s * 0.002);
        near(cmd.force, s * 42);
        cmd = step(-s * 0.01, 0.25, s * 2, s * 1000, NAN);
        near(cmd.integral, s * 1.998);         /* unwind while saturated */

        cmd = step(s, 0.25, s * 0.2, s * 800, -s * 680);
        near(cmd.requested_force, -s * 680);
        near(cmd.force, s * 550);              /* G2 shares the real slew */
        near(cmd.integral, 0);
        bp_motion_no_contact(&cmd, s * 800, 0.05);
        near(cmd.force, s * 300);              /* start from 800, not 550 */
        near(cmd.requested_force, -s * 680);
        bp_motion_no_contact(&cmd, cmd.force, 0.05);
        near(cmd.force, 0);                    /* no sign crossing */

        for (int speed = -1; speed <= 1; speed++) {
            double previous = s * 5000;
            assert(bp_motion_pi(2, 0.05, 1000, 5000, 0.25, s, speed,
                s * 0.2, previous, NAN, &cmd));
            double request = cmd.requested_force;
            for (int frame = 1; frame <= 11; frame++) {
                bp_motion_no_contact(&cmd, previous, 0.05);
                near(cmd.force, s * fmax(0, 5000 - 500 * frame));
                near(cmd.integral, 0);
                near(cmd.requested_force, request);
                previous = cmd.force;
            }
        }
    }
}

static void
invalid_samples(void)
{
    const double bad_dt[] = {0, -0.01, 0.10001, NAN, INFINITY};
    bp_motion_force_t cmd;
    for (size_t j = 0; j < sizeof(bad_dt) / sizeof(*bad_dt); j++)
        assert(!bp_motion_pi(2, bad_dt[j], 1000, 5000, 1, 1, 0, 0, 0,
            NAN, &cmd));
    assert(!bp_motion_sample_ok(1, 0.05));
    assert(!bp_motion_sample_ok(3, 0.05));
    assert(bp_motion_sample_ok(1.5, 0.1));
    assert(!bp_motion_pi(2, 0.05, 0, 5000, 1, 1, 0, 0, 0, NAN, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, NAN, 0, 0, 0, NAN, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, 1, NAN, 0, 0, NAN, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, 1, 0, NAN, 0, NAN, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, 1, 0, 0, NAN, NAN, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, 1, 0, 0, 0, INFINITY, &cmd));
    assert(!bp_motion_pi(2, 0.05, 1000, 5000, 1, DBL_MAX / 8, 0,
        0, 0, NAN, &cmd));                    /* finite accel, request overflows */
    near(cmd.requested_force, 0);
    near(cmd.force, 0);
    near(cmd.integral, 0);
}

static void
feedback(double factor, int dir, int fps)
{
    double t = 0, v = 0, target = dir * factor, mass = 65000;
    bp_motion_force_t cmd = {0};
    for (int n = 0; t < 15; n++) {
        double dt = fps ? 1.0 / fps : (n % 2 ? 1.0 / 20 : 1.0 / 120);
        double last = cmd.force;
        assert(bp_motion_pi(factor, dt, mass, 5000, 0.25, target, v,
            cmd.integral, last, NAN, &cmd));
        assert(fabs(cmd.force - last) <= cmd.limit * dt + 1e-8);
        assert(fabs(cmd.force) <= cmd.limit);
        assert(fabs(cmd.force / mass) <= 0.25 * factor * factor + 1e-12);
        v += dt * cmd.force / mass;
        t += dt;
    }
    assert(fabs(v - target) < 0.01);
}

int
main(void)
{
    const int fps[] = {20, 30, 60, 120, 0};
    limits_and_contact();
    invalid_samples();
    for (double f = 1.5; f <= 2; f += 0.5) {
        for (int dir = -1; dir <= 1; dir += 2) {
            for (size_t i = 0; i < sizeof(fps) / sizeof(*fps); i++)
                feedback(f, dir, fps[i]);
            for (int side = -1; side <= 1; side += 2) {
                double spd = dir * 3, circ = side * 20, dt = 0.05;
                double yaw_rate = dir * side * 0.4;
                double ref = (spd / circ * 360 - yaw_rate) * (f * dt);
                near(bp_motion_turn_increment(spd, circ, yaw_rate * f * dt,
                    dt, f), ref);
                near(bp_motion_limited_steer(side * 30, spd, spd / 2),
                    side * (dir > 0 ? 15 : 30));
            }
        }
    }
    return 0;
}
