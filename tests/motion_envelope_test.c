/* SPDX-License-Identifier: CDDL-1.0 */
/* Standalone C99 test of the production G2 math; no simulator or SDK. */
#include <assert.h>
#include <stdio.h>

#include "motion_envelope.h"

#ifdef BP_TEST_ROUTE_ENVELOPE
/* Optional SDK/list fixture: link the production drive_route_envelope and
 * its geometry/list dependencies (or the driving function-extraction runner).
 * Default build remains a standalone, portable math test. */
#include <stddef.h>
#include "driving.h"
int bp_xp_ver = 12000;
#endif

static void
near(double a, double b)
{
    assert(isfinite(a) && isfinite(b) && fabs(a - b) < 1e-9);
}

static motion_envelope_t
sample(int s, double actual, double force)
{
    motion_envelope_t e;
    assert(motion_envelope_begin(&e, s, 2, s * actual, 6, 0,
        1000, 2000, force));
    return e;
}

static void
braking(int s)
{
    /* Last drive frame was still adding force: 800 -> 1000 N before
     * reaching the signed 680 N brake target at 2000 N/s. */
    motion_envelope_t e = sample(s, 0, s * 800);
    near(e.b_mps2, 0.68);
    near(e.brake_force_n, -s * 680);
    near(e.reaction_s, 0.1 + (1000.0 + 680) / 2000);
    near(e.accel_plus_mps2, 1);
    double t = e.reaction_s, v = 2, w = 0.2;
    double d = v * t + t * t / 2 +
        ((v + t) * (v + t) - w * w) / (2 * 0.68);
    near(motion_envelope_distance(&e, v, w), d);
    assert(motion_envelope_bound(&e, d, w, MOTION_BOUND_END));
    assert(e.binding && !e.intervene && e.allowed_mps <= v);
    assert(v - e.allowed_mps <= (6 - w) / 16777216.0);
    near(e.distance_m, d);
    near(e.entry_mps, w);
    assert(e.boundary == MOTION_BOUND_END);

    /* Start below the envelope, intervene at it, release at crawl. */
    e = sample(s, v, s * 800);
    assert(motion_envelope_bound(&e, d, w, MOTION_BOUND_END));
    assert(e.intervene && e.actual_mps == v);
    e = sample(s, w, -s * 2000);
    near(e.reaction_s, 0.1);
    assert(motion_envelope_bound(&e, 0, w, MOTION_BOUND_REVERSE));
    near(e.allowed_mps, w);
    assert(!e.intervene);
    e = sample(s, 9, s * 1950);
    near(e.reaction_s, 0.1 + (2000.0 + 680) / 2000);
    near(e.accel_plus_mps2, 2);
    assert(motion_envelope_bound(&e, 1e5, w, MOTION_BOUND_END));
    assert(!e.binding && !e.intervene && e.allowed_mps == 6);

    /* Angular projection, controller authority, and force/mass bound.
     * Here the force limit, not nominal G1 authority, determines b. */
    assert(motion_envelope_begin(&e, s, 1.5, s * 1.1, 3, -60,
        1000, 400, 0));
    near(e.b_mps2, 0.8 * 0.5 * 0.4);
    near(e.brake_force_n, -s * 320);
    near(e.reaction_s, 0.1 + (40.0 + 320) / 400);
    near(e.accel_plus_mps2, 0.25 * 1.5 * 1.5);
    assert(motion_envelope_bound(&e, 0, 0.15, MOTION_BOUND_END));
    near(e.allowed_mps, 0.15);
    assert(e.intervene);

    /* Rollback opposite the route: keep geometry direction but account for
     * the driving force in the measured direction and oppose actual motion. */
    assert(motion_envelope_begin(&e, s, 2, -s * 1.1, 6, 0,
        1000, 2000, -s * 800));
    near(e.reaction_s, 0.1 + (1000.0 + 680) / 2000);
    near(e.actual_mps, 1.1);
    assert(motion_envelope_bound(&e, 0, 0.2, MOTION_BOUND_END));
    assert(e.intervene && e.direction == s);
    near(e.brake_force_n, s * 680);
}

static void
geometry(int s, int side)
{
    bool right = side > 0, backward = s < 0;
    int sense = s * side;
    /* Cross north in both directions, using the LONG 270 degree turn.
     * Aircraft heading at the measured position is deliberately unnecessary. */
    double start = sense > 0 ? 350 : 10;
    double end = fmod(start + sense * 270 + 360, 360);
    double r = 8, cx = side * r * cos(start * MOTION_ENV_RAD);
    double cy = -side * r * sin(start * MOTION_ENV_RAD);
    for (int offset = -10; offset <= 280; offset += 10) {
        double radial = (start - side * 90 + sense * offset) * MOTION_ENV_RAD;
        /* Lateral displacement from nominal circle must not add arc length. */
        double x = cx + 1.5 * r * sin(radial);
        double y = cy + 1.5 * r * cos(radial);
        double remaining = 270 - fmax(0, fmin(270, offset));
        near(motion_envelope_arc_m(0, 0, start, end, r, right, backward,
            true, x, y), r * remaining * MOTION_ENV_RAD);
    }
    near(motion_envelope_arc_m(0, 0, start, end, r, right, backward,
        false, 0, 0), r * 270 * MOTION_ENV_RAD);
    assert(isnan(motion_envelope_arc_m(0, 0, start, end, r, right, backward,
        true, cx, cy)));

    /* A later tight curve wins over a distant end. Entry uses BOTH the
     * angular and lateral limits (here angular wins); accumulated distance
     * starts with the real projected straight remainder, not its full length. */
    motion_envelope_t e = sample(s, 0, 0);
    double d = 0;
    assert(motion_envelope_straight(&d, s, 10, 0, 0, 0, true, 100, s * 9));
    near(d, 1);
    assert(motion_envelope_straight(&d, s, 2, 0, 0, 0, false, 100, s * 9));
    assert(motion_envelope_curve(&e, &d, 2, 5, 10, 0, 0, start, end,
        r, right, false, 0, 0));
    assert(e.boundary == MOTION_BOUND_CURVE);
    near(e.distance_m, 3);
    near(e.entry_mps, 2 * 5 * MOTION_ENV_RAD * r);
    near(d, 3 + r * 270 * MOTION_ENV_RAD);
    assert(motion_envelope_bound(&e, d + 100, 0.2, MOTION_BOUND_END));
    assert(e.boundary == MOTION_BOUND_CURVE);

    e = sample(s, 0, 0);
    d = 0;
    assert(motion_envelope_curve(&e, &d, 2, 90, 0.125, 0, 0, start, end,
        r, right, true, 0, 0));
    near(e.distance_m, 0);
    near(e.allowed_mps, 2); /* current curve: lateral constraint wins */
    assert(motion_envelope_bound(&e, 0, 0.2, MOTION_BOUND_REVERSE));
    near(e.allowed_mps, 0.2);
    assert(e.boundary == MOTION_BOUND_REVERSE);
}

static void
watchdog(int s)
{
    motion_envelope_watch_t watch = {0};
    motion_envelope_t e = sample(s, 0.36, -s * 680);
    assert(motion_envelope_bound(&e, 0, 0.2, MOTION_BOUND_END));
    assert(motion_envelope_monitor(&watch, &e, 0, 0.02) == MOTION_ENV_OK);
    assert(motion_envelope_monitor(&watch, &e, 0, 0.02) ==
        MOTION_ENV_OVERSPEED);
    e = sample(s, 0.3, -s * 680);
    assert(motion_envelope_bound(&e, 0, 0.2, MOTION_BOUND_END));
    for (int i = 0; i < 6; i++)
        assert(motion_envelope_monitor(&watch, &e, -s * 679, 0.1) ==
            MOTION_ENV_OK); /* ramp still short of target */
    assert(motion_envelope_monitor(&watch, &e, s * 680, 0.1) == MOTION_ENV_OK);
    assert(motion_envelope_monitor(&watch, &e, e.brake_force_n, 0.09) ==
        MOTION_ENV_OK);
    assert(motion_envelope_monitor(&watch, &e, e.brake_force_n, 0.07) ==
        MOTION_ENV_OK);
    assert(motion_envelope_monitor(&watch, &e, e.brake_force_n, 0.09) ==
        MOTION_ENV_NO_DECEL);
    e.actual_mps -= 0.001;
    assert(motion_envelope_monitor(&watch, &e, e.brake_force_n, 0.1) ==
        MOTION_ENV_OK);
    assert(motion_envelope_monitor(&watch, &e, NAN, 0.1) == MOTION_ENV_INVALID);
    assert(motion_envelope_monitor(&watch, &e, 0, 0) == MOTION_ENV_INVALID);
    assert(motion_envelope_monitor(&watch, &e, 0, 0.101) == MOTION_ENV_INVALID);
}

#ifdef BP_TEST_ROUTE_ENVELOPE
static void
route_query(int s)
{
    vehicle_t veh = {0};
    vehicle_pos_t pos = {0};
    seg_t legs[4] = {0};
    list_t segs;
    motion_envelope_t e;
    veh.use_rear_pos = B_TRUE;
    veh.fixed_z_off = -2;
    veh.max_fwd_ang_vel = 10;
    veh.max_rev_ang_vel = 15;
    veh.max_centr_accel = 15;
    pos.pos.y = 2 + s * 9;
    list_create(&segs, sizeof (seg_t), offsetof(seg_t, node));
    for (int i = 0; i < 4; i++) {
        legs[i].type = SEG_TYPE_STRAIGHT;
        legs[i].backward = s < 0;
        list_insert_tail(&segs, &legs[i]);
    }
    legs[0].len = 10;
    legs[0].end_pos.y = s * 10;
    legs[1].start_pos = legs[0].end_pos;
    legs[1].len = 1;
    legs[1].end_pos.y = s * 11;
    legs[2].type = SEG_TYPE_TURN;
    legs[2].start_pos = legs[1].end_pos;
    legs[2].turn.right = B_TRUE;
    legs[2].turn.r = 4;
    legs[2].end_hdg = s > 0 ? 90 : 270;
    legs[2].end_pos.x = 4;
    legs[2].end_pos.y = s * 15;
    legs[3].backward = s > 0;
    legs[3].len = NAN; /* deliberately invalid beyond the direction change */

    for (int i = 0; i < 2; i++) {
        assert(drive_route_envelope(&pos, &veh, &segs, 2, 0, 6, 0,
            1000, 2000, 0, &e));
        assert(e.boundary == MOTION_BOUND_CURVE);
        near(e.distance_m, 2);
        near(e.entry_mps, 8 * MOTION_ENV_RAD * (s > 0 ? 10 : 15));
        assert(list_head(&segs) == &legs[0]);
        assert(list_next(&segs, &legs[1]) == &legs[2]);
    }
    /* At the first line's endpoint, drive_segs could release the head.
     * G2 instead sees the reversal at the same position on every query. */
    legs[1].backward = !legs[0].backward;
    pos.pos.y += s;
    assert(drive_route_envelope(&pos, &veh, &segs, 2, s * 0.3, 6, 0,
        1000, 2000, 0, &e));
    assert(e.boundary == MOTION_BOUND_REVERSE && e.intervene);
    near(e.distance_m, 0);
    near(e.allowed_mps, 0.2);
    assert(list_head(&segs) == &legs[0]);
    for (int i = 0; i < 4; i++)
        list_remove(&segs, &legs[i]);
    assert(!drive_route_envelope(&pos, &veh, &segs, 2, 0, 6, 0,
        1000, 2000, 0, &e));
    list_destroy(&segs);
}
#endif

int
main(void)
{
    for (int s = -1; s <= 1; s += 2) {
        braking(s);
        for (int side = -1; side <= 1; side += 2)
            geometry(s, side);
        watchdog(s);
#ifdef BP_TEST_ROUTE_ENVELOPE
        route_query(s);
#endif
    }
    motion_envelope_t e;
    assert(!motion_envelope_begin(&e, 1, 2, 0, 6, 90, 1000, 1000, 0));
    assert(!motion_envelope_begin(&e, 1, 2, 0, 6, -100, 1000, 1000, 0));
    assert(!motion_envelope_begin(&e, 1, 2, 0, 6, 0, 0, 1000, 0));
    assert(!motion_envelope_begin(&e, 1, 2, 0, 6, 0, 1000, 0, 0));
    assert(!motion_envelope_begin(&e, 1, 2, NAN, 6, 0, 1000, 1000, 0));
    assert(!motion_envelope_begin(&e, 1, 2, 0, 6, 0, 1000, 1000, INFINITY));
    assert(!motion_envelope_begin(&e, 1, 1, 0, 6, 0, 1000, 1000, 0));
    assert(!motion_envelope_begin(&e, 1, 3, 0, 6, 0, 1000, 1000, 0));
    e = sample(1, 0, 0);
    assert(!motion_envelope_bound(&e, INFINITY, 0, MOTION_BOUND_END));
    assert(!motion_envelope_bound(&e, 0, NAN, MOTION_BOUND_END));
    puts("motion envelope: passed");
    return 0;
}
