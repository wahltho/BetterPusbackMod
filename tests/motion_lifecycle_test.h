/* A planar dry plant: the controller can influence motion only through its
 * emitted axial force, nosewheel and orientation correction. No target
 * speeds or planned poses are fed back as observations. */
static tug_info_t info;
static tug_t tug;

static void close_to(double got, double wanted, double eps)
{
    if (!(isfinite(got) && fabs(got - wanted) <= eps)) {
        fprintf(stderr, "got %.17g expected %.17g +/- %.9g\n", got, wanted, eps);
        abort();
    }
}

static void reset_controller(double factor, bool_t backward, double length)
{
    memset(&bp, 0, sizeof(bp)); memset(&drs, 0, sizeof(drs));
    memset(&bp_telem, 0, sizeof(bp_telem)); memset(&tug, 0, sizeof(tug));
    memset(&push_manual, 0, sizeof(push_manual));
    bp_started = B_TRUE; cfg_ignore_park_break = parking_brake = B_FALSE;
    coupled = slave_mode = fast_handling = op_complete = B_FALSE;
    bp.step = PB_STEP_STARTING; bp.step_start_t = 90;
    bp.last_t = bp.cur_t = drs.sim_time = 100; bp.d_t = 0.02;
    bp.last_hdg = NAN;
    bp.veh = vehicle(); bp.automatic_push_speed_factor = factor;
    bp.acf.nw_z = -10; bp.acf.nw_len = 1.2; bp.acf.main_z = 0;
    bp.last_seg_is_back = backward;
    info = (tug_info_t){.max_tow_fwd_speed = 4, .max_tow_rev_speed = 1.11,
        .max_fwd_speed = 5, .max_rev_speed = 2, .mass = 6000,
        .max_TE = 90000, .tug_name = "dry-tug", .max_steer = 60,
        .lift_wall_loc = LIFT_WALL_CENTER, .lift_wall_z = 0.5};
    tug.info = &info; tug.veh = vehicle(); tug.veh.wheelbase = 4;
    tug.TE_override = B_TRUE; bp_ls.tug = &tug;
    copy_route(&tug.segs, NULL, 0);
    seg_t s = straight(backward, length);
    copy_route(&bp.segs, &s, 1);
    drs.acf_mass = 60000; drs.gear_on_ground = drs.nw_steer_on = 1;
    brake_writes = quaternion_writes = draws = animations = manual_calls = 0;
    complete_messages = 0; last_d_hdg = 0;
#if !FROZEN
    motion_reset();
#endif
}

static void finish_fixture(void)
{
    clear_route(&bp.segs); clear_route(&tug.segs);
}

static void tick(double dt)
{
    drs.sim_time += dt;
    drs.axial_force = drs.rot_force_M = drs.rot_force_N = 0;
    last_d_hdg = 0;
    assert(bp_run(0, 0, 0, NULL) == -1);
}

static void arm_push(void)
{
    tick(0.02);
    assert(bp.step == PB_STEP_PUSHING && !op_complete);
}

static double frame_dt(int cadence, unsigned n)
{
    const double rates[] = {20, 30, 60, 120};
    return 1.0 / rates[cadence == 4 ? n % 4 : (unsigned)cadence];
}

static void integrate(double dt)
{
    double v = bp.cur_pos.spd - drs.axial_force / drs.acf_mass * dt;
    /* A stationary brake application captures the remaining crawl. */
    if (drs.lbrake == 1 && drs.rbrake == 1 && fabs(v) < 0.08)
        v = 0;
    double h = normalize_hdg(drs.hdg + RAD2DEG(v * dt *
        tan(DEG2RAD(drs.tire_steer_cmd)) / bp.veh.wheelbase) + last_d_hdg);
    vect2_t d = hdg2dir(h);
    drs.local_vx = v * d.x; drs.local_vz = -v * d.y;
    drs.local_x += drs.local_vx * dt; drs.local_z += drs.local_vz * dt;
    drs.hdg = normalize_hdg((float)h); /* X-Plane true heading dataref is float. */
}

/* Use %a so equality of the two independent executables covers every
 * nonsymbolic bit, including signed zero, without struct padding. */
static void trace_sample(FILE *out)
{
    fprintf(out, "%d %d %u %u %lu", (int)bp.step, (int)op_complete,
        brake_writes, complete_messages, (unsigned long)list_count(&bp.segs));
    const double data[] = {bp.last_force, drs.axial_force, drs.rot_force_N,
        drs.rot_force_M, drs.tire_steer_cmd, drs.lbrake, drs.rbrake,
        tug.pos.hdg, tug.pos.spd, tug.cur_steer, last_d_hdg, bp.last_mis_hdg,
        bp.cur_pos.pos.y, bp.d_pos.spd / bp.d_t, bp_telem.decelerating,
        bp_telem.target_speed_raw_mps, bp_telem.target_speed_limited_mps};
    for (size_t i = 0; i < sizeof(data)/sizeof(*data); i++)
        fprintf(out, " %a", data[i]);
    fputc('\n', out);
}

static void reference_traces(const char *path)
{
    FILE *out = fopen(path, "wb"); assert(out);
    for (int back = 0; back <= 1; back++)
    for (int side = -1; side <= 1; side++)
    for (int cadence = 0; cadence <= 4; cadence++) {
        reset_controller(1, back, 200); arm_push();
        tug.pos.hdg = normalize_hdg(12 * side); tug.cur_steer = 7 * side;
        drs.local_x = 4 * side;
        for (unsigned i = 0; i < 150; i++) {
            double dt = frame_dt(cadence, i);
            drs.rwy_friction = (i / 30) % 3;
            drs.gear_on_ground = i != 48;
            drs.rbrake = i == 49 ? 0.3 : 0;
            tick(dt); trace_sample(out); integrate(dt);
        }
        finish_fixture();
    }
    for (int ignore = 0; ignore < 2; ignore++)
    for (int slave = 0; slave < 2; slave++) {
        reset_controller(1, B_FALSE, 4); arm_push();
        slave_mode = slave; cfg_ignore_park_break = ignore;
        bp.step = PB_STEP_STOPPING; bp.step_start_t = bp.cur_t;
        while (bp.step == PB_STEP_STOPPING && bp.cur_t < 104) {
            tick(0.05); trace_sample(out);
        }
        assert(bp.step == PB_STEP_STOPPED && complete_messages == 1);
        assert((brake_writes > 1) == (!slave && !ignore));
        finish_fixture();
    }
    /* Exercise both entry points even in the frozen executable. */
    telemetry_enabled = 0;
    telemetry_start(); telemetry_refresh_speed_limits();
    telemetry_enabled = 1;
    assert(fclose(out) == 0);
    (void)recon_handler; (void)close_to;
}

#if !FROZEN
static void config_and_start(void)
{
    static const char *const bad[] = {"", " ", "2x", "1.5garbage", "2 1",
        "nan", "inf", "-inf", "1e9999", "0x1p9999", "-2", "1.5001", "3"};
    for (size_t i = 0; i < sizeof(bad)/sizeof(*bad); i++) {
        config_text = bad[i]; assert(bp_pushback_speed_factor() == 1);
    }
    for (int i = -1; i <= 4; i++)
        assert(bp_pushback_speed_choice(i) == (i == 1 ? 1.5 : i == 2 ? 2 : 1));
    config_text = " 1.5 \t"; assert(bp_pushback_speed_factor() == 1.5);
    config_text = "2.000000"; assert(bp_pushback_speed_factor() == 2);
    config_text = NULL; bp_conf = NULL; assert(bp_pushback_speed_factor() == 1);
    bp_conf = &config_storage; config_readable = B_FALSE;
    assert(bp_pushback_speed_factor() == 1); config_readable = B_TRUE;
    reset_controller(1, B_TRUE, 1000); bp_started = can_start = B_FALSE;
    config_reads = 0; config_factor = 2;
    assert(!bp_start() && config_reads == 0 && bp.automatic_push_speed_factor == 1);
    can_start = B_TRUE;
    assert(bp_start() && config_reads == 1 && bp.automatic_push_speed_factor == 2);
    config_factor = 1.5;
    assert(bp_start() && config_reads == 1 && bp.automatic_push_speed_factor == 2);
    bp.step = PB_STEP_WAITING4OK2DISCO;
    motion.previous_tug_speed = 4;
    assert(recon_handler(NULL, 0, NULL) && bp.step == PB_STEP_GRABBING);
    assert(config_reads == 1 && bp.automatic_push_speed_factor == 2);
    assert(isnan(motion.previous_tug_speed) && !motion.armed);
    bp_started = B_FALSE;
    assert(bp_start() && config_reads == 2 && bp.automatic_push_speed_factor == 1.5);
    finish_fixture();
}

static void heading_history(double f, int dir, int side)
{
    reset_controller(f, dir < 0, 500); arm_push();
    drs.local_vz = -dir * 0.55;
    tug.cur_steer = side * 18; tug.steer_override = B_TRUE;
    tick(0.025);
    double previous = tug.pos.spd;
    close_to(previous, dir * 0.55, 1e-12);
    /* A position-only draw overwrites public speed but must not advance
     * heading or become the next frame's private history. */
    double start_h = tug.pos.hdg;
    bp.cur_pos.spd = -dir * 3; draw_tugs();
    same_double(tug.pos.hdg, start_h);
    assert(fabs(tug.pos.spd - previous) > 1);
    same_double(motion.previous_tug_speed, previous);
    tug.cur_steer = side * 18; tug.steer_override = B_TRUE;
    double physical = dir * 0.9 * cos(DEG2RAD(drs.tire_steer_cmd));
    drs.local_vz = -dir * 0.9;
    tick(0.025);
    assert(!bp_motion_fault_held());
    const double r = tug.veh.wheelbase / tan(DEG2RAD(side * 18));
    double ref_speed = previous / f - dir * TUG_MAX_ACCEL * bp.d_t * f;
    close_to(anim_speed, ref_speed * f, 1e-11);
    close_to(rel_hdg(draw_heading, anim_heading),
        RAD2DEG(ref_speed * bp.d_t * f / r), 1e-10);
    close_to(rel_hdg(anim_heading, tug.pos.hdg), RAD2DEG(physical * bp.d_t / r), 1e-10);
    close_to(motion.previous_tug_speed, physical, 1e-11);
    finish_fixture();
}

static void fault_and_hold(double f, int fault)
{
    reset_controller(f, B_FALSE, 200); arm_push();
    tick(0.05); assert(motion_factor() == f && bp.last_force > 0);
    if (fault <= 2) {
        unsigned before = quaternion_writes, old_draws = draws, old_anim = animations;
        if (fault == 2) drs.local_x = NAN;
        tick(fault == 0 ? -0.02 : fault == 1 ? 0.3 : 0.05);
        assert(before == quaternion_writes && old_draws == draws && old_anim == animations);
        assert(bp.last_force == 0 && bp.motion_integral == 0 && drs.axial_force == 0);
        assert(isnan(motion.previous_tug_speed));
        drs.local_x = 0;
        tick(0.05); assert(draws == old_draws && !bp.last_force);
    } else if (fault == 3) {
        drs.gear_on_ground = 0; tick(0.05); drs.gear_on_ground = 1;
    } else {
        /* No pose feedback, even for a 1.5x final crawl. */
        drs.local_z = -199.999;
        for (int i = 0; i < 45 && !bp_motion_fault_held(); i++) tick(0.05);
    }
    assert(bp_motion_fault_held() && !op_complete && !bp_request_resume());
    assert(list_count(&bp.segs) == 1);
    drs.local_vz = -0.2;
    assert(bp_stop()); tick(0.05);
    assert(bp.step == PB_STEP_PUSHING && !op_complete && !parking_brake);
    assert(!isfinite(bp_telem.target_speed_limited_mps) || bp_telem.target_speed_limited_mps == 0);
    drs.local_vz = 0; tick(0.05);
    assert(bp.step == PB_STEP_STOPPING && bp.stop_from_pause_hold && !op_complete);
    for (int i = 0; i < 50 && bp.step == PB_STEP_STOPPING; i++) {
        tick(0.05);
        assert(!bp_telem.command_active || bp_telem.target_speed_limited_mps == 0);
    }
    assert(bp.step == PB_STEP_STOPPED && complete_messages == 1 && !parking_brake);
    finish_fixture();
}

typedef struct { double target, request, steering; } arc_sample_t;

static arc_sample_t arc_sample(double f, double dt, int dir, int side)
{
    const double r = 25, a = DEG2RAD(35);
    reset_controller(f, dir < 0, 500); arm_push();
    seg_t *s = list_head(&bp.segs);
    s->type = SEG_TYPE_TURN; s->turn.right = side > 0; s->turn.r = r;
    s->end_pos = VECT2(side*r, dir*r); s->end_hdg = normalize_hdg(side*dir*90);
    drs.local_x = side*r*(1-cos(a)); drs.local_z = -dir*r*sin(a);
    /* Small real tracking error supplies a nonzero steering demand. */
    drs.hdg = normalize_hdg(side * (dir*35 + 2));
    tug.pos.hdg = normalize_hdg(drs.hdg + side*2);
    bp.last_pos.hdg = drs.hdg;
    vect2_t d = hdg2dir(drs.hdg);
    drs.local_vx = 0.4*dir*f*d.x; drs.local_vz = -0.4*dir*f*d.y;
    tick(dt);
    assert(!bp_motion_fault_held() && bp.step == PB_STEP_PUSHING && !op_complete);
    assert(list_count(&bp.segs) == 1 && bp_telem.command_active);
    arc_sample_t sample = {bp_telem.target_speed_limited_mps,
        bp_telem.applied_steer_cmd_deg, tug.cur_steer};
    /* Compare both outputs in the aircraft's longitudinal frame. */
    if (f == 1)
        sample.target *= cos(DEG2RAD(rel_hdg(bp.cur_pos.hdg, tug.pos.hdg)));
    assert(dir * sample.target > 0.4*f && fabs(sample.request) > 0.01);
    finish_fixture(); return sample;
}

static void curvature_and_zero_steer(double f, int dir, int side)
{
    arc_sample_t ref = arc_sample(1, 0.025*f, dir, side);
    arc_sample_t fast = arc_sample(f, 0.025, dir, side);
    close_to(fast.target, f*ref.target, 1e-9);
    close_to(fast.request, ref.request, 1e-9);
    close_to(fast.steering, ref.steering, 1e-9);
    reset_controller(f, dir < 0, 500); arm_push();
    tug.pos.spd = dir*0.45*f;
    assert(tug.cur_steer == 0 && bp.d_pos.hdg == 0 && motion_factor() == f);
    unsigned writes = quaternion_writes;
    turn_nosewheel(0);
    assert(!bp_motion_fault_held() && quaternion_writes == writes+1);
    assert(drs.tire_steer_cmd == 0 && last_d_hdg == 0 && tug.cur_steer == 0);
    finish_fixture();
}

static void terminal_preview_gates(void)
{
    for (int dir = -1; dir <= 1; dir += 2) {
        reset_controller(2, dir < 0, 4);
        seg_t *s = list_head(&bp.segs);
        vehicle_pos_t p = {.pos = {0, dir}, .hdg = 0, .spd = dir};
        double command = 24;
        assert(!motion_terminal_steer(s, &p, &command) && command == 24);
        p.pos.y = 3*dir;
        seg_t *next = calloc(1, sizeof(*next)); assert(next);
        list_insert_tail(&bp.segs, next);
        assert(!motion_terminal_steer(s, &p, &command) && command == 24);
        list_remove(&bp.segs, next); free(next);
        p.spd = -dir;
        assert(!motion_terminal_steer(s, &p, &command) && command == 24);
        p.spd = dir;
        assert(motion_terminal_steer(s, &p, &command));
        close_to(command, 12, 1e-12);
        /* Near the endpoint geometrically, but at the start of a nearly
         * complete circle: do not anticipate the last few degrees here. */
        s->type = SEG_TYPE_TURN; s->turn.right = dir > 0; s->turn.r = 20;
        s->end_hdg = 359; s->end_pos = s->start_pos; p.pos = s->start_pos;
        assert(!motion_terminal_steer(s, &p, &command) && command == 12);
        finish_fixture();
    }
}

static void lifecycle_tests(void)
{
    config_and_start();
    terminal_preview_gates();
    for (double f = 1.5; f <= 2; f += 0.5) {
        for (int dir = -1; dir <= 1; dir += 2)
        for (int side = -1; side <= 1; side += 2) {
            heading_history(f, dir, side);
            curvature_and_zero_steer(f, dir, side);
            reset_controller(f, dir < 0, 500); arm_push();
            tug.pos.hdg = normalize_hdg(60 * side);
            drs.tire_steer_cmd = 60 * side;
            for (int poor = 0; poor <= 1; poor++) {
                drs.rwy_friction = poor ? RWY_FRICTION_POOR : RWY_FRICTION_MED;
                push_at_speed(dir * 4, bp.veh.max_accel, B_FALSE, B_FALSE);
                close_to(bp_telem.target_speed_limited_mps, dir * 0.5 *
                    (poor ? MAX_SPEED_POOR_FRICTION : MAX_SPEED_MED_FRICTION), 1e-9);
            }
            finish_fixture();
        }
        for (int fault = 0; fault <= 4; fault++) fault_and_hold(f, fault);
        for (int gate = 0; gate < 5; gate++) {
            reset_controller(f, B_FALSE, 300);
            coupled = gate == 0; slave_mode = gate == 1;
            push_manual.active = gate == 2 || gate == 3;
            push_manual.with_yoke = gate == 2; push_manual.requested = gate == 4;
            arm_push(); tick(0.025);
            assert(motion_factor() == 1 && !motion.armed);
            if (push_manual.active) assert(manual_calls == 1);
            finish_fixture();
        }
        reset_controller(f, B_TRUE, 300); arm_push();
        assert(bp_request_pause()); tick(0.05);
        assert(bp.pause_hold && bp_telem.target_speed_limited_mps == 0);
        parking_brake = B_TRUE; assert(!bp_request_resume()); parking_brake = B_FALSE;
        assert(bp_request_resume()); tick(0.05);
        assert(!bp.pause_hold && bp_telem.target_speed_limited_mps < 0);
        drs.taxi_light_on = 1;
        for (int i = 0; i < 50; i++) tick(0.05);
        assert(!bp_motion_fault_held() && bp_telem.target_speed_limited_mps == 0);
        drs.taxi_light_on = 0; drs.lbrake = 0.5; tick(0.05);
        assert(drs.axial_force == 0 && !bp.last_force && !bp.motion_integral);
        assert(list_count(&bp.segs) == 1);
        finish_fixture();
        reset_controller(f, B_FALSE, 10);
        seg_t *reverse = calloc(1, sizeof(*reverse)); assert(reverse);
        *reverse = straight(B_TRUE, 100);
        reverse->start_pos.y = 10; reverse->end_pos.y = -90;
        list_insert_tail(&bp.segs, reverse); arm_push();
        drs.local_z = -10; drs.local_vz = -0.1;
        tick(0.05); assert(list_head(&bp.segs) == reverse);
        assert(bp.reverse_t == bp.cur_t && bp_telem.target_speed_limited_mps == 0);
        for (int i = 0; i < 82; i++) {
            tick(0.05);
            assert(bp.reverse_t > 0 && bp_telem.target_speed_limited_mps == 0);
        }
        drs.local_vz = 0; tick(0.05);
        assert(bp.reverse_t == 0 && bp_telem.target_speed_limited_mps < 0);
        unsigned before = animations;
        drs.sim_paused = 1; tick(10);
        assert(animations == before && !bp_motion_fault_held());
        drs.sim_paused = 0; tick(0.05);
        assert(animations == before && !bp.last_force && !bp.motion_integral);
        tick(0.05);
        assert(animations == before+1 && !bp_motion_fault_held());
        finish_fixture();
        for (int friction = RWY_FRICTION_MED; friction <= RWY_FRICTION_POOR; friction++) {
            reset_controller(f, B_FALSE, 300); arm_push();
            drs.rwy_friction = friction; tick(0.025);
            double lim = friction == RWY_FRICTION_MED ? 2 : 1.11;
            assert(bp_telem.target_speed_raw_mps == f*4);
            same_double(bp_telem.target_speed_limited_mps, lim);
            finish_fixture();
        }
    }
}

#endif
typedef struct { double time, peak, end, lateral, heading; } trip_t;
static unsigned pose_misses;

static trip_t trip(double f, int back, double length, int cadence, FILE *trace, FILE *prefix)
{
    reset_controller(f, back, length);
    /* AP88-sized physical profile after tug load, not a fast physical cap. */
    bp.veh.max_fwd_spd = info.max_tow_fwd_speed = 1.7778;
    info.max_tow_rev_speed = 1.7778;
    arm_push();
    trip_t t = {0};
    double start = bp.cur_t;
#if !FROZEN
    double prev = 0;
#endif
    for (unsigned n = 0; n < 90000; n++) {
        double dt = frame_dt(cadence, n);
        /* All trip fixtures start at zero, point north and have their
         * reference at the main axle. End the raw-reference oracle BEFORE
         * the first sample whose planner can take the unwritten crawl exit. */
        if (length - (back ? drs.local_z : -drs.local_z) < CRAWL_SPEED(bp_xp_ver, &bp.veh))
            prefix = NULL;
        tick(dt);
        if (prefix) trace_sample(prefix);
        if (trace && cadence == 0 && length == 300)
            trace_sample(trace);
#if !FROZEN
        if (bp_motion_fault_held()) {
            fprintf(stderr, "F=%g length=%g back=%d cadence=%d t=%g fault: %s\n",
                f, length, back, cadence, bp.cur_t-start, motion.fault);
            abort();
        }
        if (f > 1 && bp_telem.command_active) {
            double cap = FORCE_PER_TON * drs.acf_mass / 1000;
            assert(fabs(bp.last_force) <= cap && fabs(bp.last_force-prev) <= cap*bp.d_t+1e-8);
            double rad = DEG2RAD(drs.tire_steer_cmd);
            close_to(-drs.axial_force, bp.last_force * cos(rad), 1e-7);
            close_to(drs.rot_force_N, -bp.acf.nw_z * bp.last_force * sin(rad), 1e-7);
            close_to(drs.rot_force_M, -drs.axial_force * bp.acf.nw_len, 1e-7);
        }
        prev = bp.last_force;
#endif
        t.peak = fmax(t.peak, (back ? -1 : 1) * bp.cur_pos.spd);
        if (bp.step == PB_STEP_STOPPING && op_complete &&
            fabs(bp.cur_pos.spd) < SPEED_COMPLETE_THRESH && brake_writes > 1) {
            t.time = bp.cur_t - start; t.end = (back ? -1 : 1) * bp.cur_pos.pos.y;
            fprintf(stderr, "trip F=%g cadence=%d back=%d length=%g time=%.4f "
                "end=%.5f x=%.5f hdg=%.5f peak=%.5f\n", f, cadence, back,
                length, t.time, t.end, bp.cur_pos.pos.x, rel_hdg(0, bp.cur_pos.hdg), t.peak);
            assert(list_count(&bp.segs) == 0);
            t.lateral = bp.cur_pos.pos.x; t.heading = rel_hdg(0, bp.cur_pos.hdg);
            if (f > 1 && (fabs(t.end-length) > 0.5 || fabs(t.lateral) > 0.01 ||
                fabs(t.heading) > 0.01)) pose_misses++;
            break;
        }
        integrate(dt);
    }
    assert(t.time > 0); finish_fixture(); return t;
}

#include "curved_route_regression.h"

static void closed_loop_tests(const char *trace, const char *prefix)
{
    FILE *out = fopen(trace, "ab"); assert(out);
    FILE *raw = fopen(prefix, "ab"); assert(raw);
    for (int cadence = 0; cadence < 5; cadence++)
    for (int back = 0; back < 2; back++)
    for (int short_route = 0; short_route < 2; short_route++) {
        double length = short_route ? 4 : 300;
        fprintf(raw, "trip %d %d %.0f\n", cadence, back, length);
        trip_t base = trip(1, back, length, cadence, out, raw);
        fprintf(out, "trip %d %d %a %a %a %a %a\n", cadence, back,
            base.time, base.peak, base.end, base.lateral, base.heading);
#if !FROZEN
        for (double f = 1.5; f <= 2; f += 0.5) {
            trip_t fast = trip(f, back, length, cadence, NULL, NULL);
            /* Compare error to the planned endpoint, not distance from an
             * overshooting reference. Candidate hard limits apply regardless. */
            assert(fabs(fast.end-length) <= fmax(0.5, fabs(base.end-length)));
            if (!short_route) {
                close_to(fast.time * f / base.time, 1, 0.10);
                close_to(fast.peak / base.peak, f, 0.10 * f);
            }
        }
#else
        (void)base;
#endif
    }
    for (size_t r = 0; r < sizeof(curved_routes)/sizeof(curved_routes[0]); r++)
    for (int cadence = 0; cadence < 5; cadence++) {
        curved_result_t ref = curved_route_trip(&curved_routes[r], 1, cadence, out);
#if !FROZEN
        for (double f = 1.5; f <= 2; f += 0.5) {
            curved_result_t fast = curved_route_trip(&curved_routes[r], f, cadence, NULL);
            assert(fast.time < ref.time);
            /* Compare endpoint errors, not offsets from a possibly imperfect
             * reference stop. Allow half a metre for the faster stop. */
            assert(fast.end_error <= ref.end_error + 0.5);
            if (!curved_routes[r].turn_first)
                assert(fast.heading_error <= 1.0);
            else
                assert(fast.heading_error <= ref.heading_error + 1.0);
        }
#else
        (void)ref;
#endif
    }
    assert(fclose(raw) == 0 && fclose(out) == 0 && !pose_misses);
}
