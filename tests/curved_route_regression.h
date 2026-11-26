/* Saved reverse pushes, WGS84 local east/north metres from segment 1 start.
 * Start positions, headings, aircraft mass and AST-3F selection are based on
 * the three flight captures; dynamics come from the planar test fixture. */
static const struct curved_route_case {
    const char *gate;
    vect2_t start, join, end;
    double aircraft_heading, start_heading, end_heading, radius, mass;
    bool_t turn_first;
} curved_routes[] = {
    {"KSEA N16", {0.282912,-0.041545}, {-4.317474,4.027028}, {0.152425,60.737481},
        133.24, 133.2, 236.2, 36.345, 49450, B_FALSE},
    {"BGTL Military Ramp 1", {0.020593,-0.147731}, {4.573230,-14.951988}, {-22.788358,-64.326597},
        342.73111, 342.7, 74.7, 39.237, 53737, B_FALSE},
    {"KBNA C4", {0.057836,-0.002976}, {70.475237,24.085662}, {81.100778,14.043968},
        188.693085, 188.7, 313.2, 42.079, 52416, B_TRUE}
};

typedef struct { double time, end_error, heading_error; } curved_result_t;

static curved_result_t curved_route_trip(const struct curved_route_case *r, double f,
    int cadence, FILE *reference_trace)
{
    reset_controller(f, B_TRUE, 1);
    seg_t seg[2] = {0};
    const int turn = r->turn_first ? 0 : 1, line = 1-turn;
    seg[0].start_hdg = r->start_heading;
    seg[0].end_hdg = seg[1].start_hdg = r->turn_first ? r->end_heading : r->start_heading;
    seg[1].end_hdg = r->end_heading;
    seg[0].end_pos = seg[1].start_pos = r->join;
    seg[1].end_pos = r->end;
    seg[0].backward = seg[1].backward = seg[1].user_placed = B_TRUE;
    seg[line].type = SEG_TYPE_STRAIGHT;
    seg[line].len = vect2_dist(seg[line].start_pos, seg[line].end_pos);
    seg[turn].type = SEG_TYPE_TURN;
    seg[turn].turn.r = r->radius;
    clear_route(&bp.segs); copy_route(&bp.segs, seg, 2);

    drs.acf_mass = r->mass;
    bp.acf.nw_z = (13.41-60.34)*0.3048;
    bp.acf.main_z = (64.51-60.34)*0.3048;
    bp.acf.tirrad = 1.14*0.3048; bp.acf.nw_len = 1.3716;
    bp.veh.wheelbase = bp.acf.main_z-bp.acf.nw_z;
    bp.veh.fixed_z_off = -bp.acf.main_z; bp.veh.use_rear_pos = B_TRUE;
    bp.veh.max_steer = 78;
    info.mass = 13000; info.max_TE = 480000;
    info.front_z = 0.09; info.rear_z = -2.9; info.lift_wall_z = -0.96;
    info.lift_wall_loc = LIFT_WALL_FRONT;
    info.max_fwd_speed = info.max_tow_fwd_speed = tug.veh.max_fwd_spd = 6;
    info.max_rev_speed = info.max_tow_rev_speed = tug.veh.max_rev_spd = 3;
    tug.veh.wheelbase = 5; tug.veh.fixed_z_off = info.rear_z;
    tug.veh.max_fwd_ang_vel = tug.veh.max_rev_ang_vel = 20;
    tug.veh.max_centr_accel = tug.veh.max_decel = 0.5; tug.veh.max_accel = 1;
    drs.local_x = r->start.x; drs.local_z = -r->start.y;
    bp.last_pos.pos = r->start;
    drs.hdg = bp.last_pos.hdg = tug.pos.hdg = r->aircraft_heading;
    bp.step_start_t = bp.cur_t - PB_START_DELAY + 0.03;
    tick(0.02);
    assert(bp.step == PB_STEP_STARTING);
    assert(r->turn_first ? isnan(bp.last_hdg) : bp.last_hdg == r->end_heading);
    arm_push();

    const double start = bp.cur_t;
    double peak = 0;
    for (unsigned n = 0; ; n++) {
        const double dt = frame_dt(cadence, n);
        tick(dt);
        peak = fmax(peak, fabs(bp.cur_pos.spd));
        if (reference_trace) trace_sample(reference_trace);
#if !FROZEN
        if (motion.fault) fprintf(stderr, "%s %.1fx cadence %d at %.2fs: %s\n",
            r->gate, f, cadence, bp.cur_t-start, motion.fault);
        assert(!motion.fault);
#endif
        assert(bp.cur_t-start < 180);
        if (!list_count(&bp.segs) && bp.step == PB_STEP_STOPPING && op_complete &&
            fabs(bp.cur_pos.spd) < SPEED_COMPLETE_THRESH && drs.lbrake == 1 && drs.rbrake == 1)
            break;
        integrate(dt);
    }
    vehicle_pos_t end = corr_acf_pos();
    curved_result_t result = {bp.cur_t-start,
        vect2_dist(veh_pos2fixed_pos(&end, &bp.veh), r->end),
        fabs(rel_hdg(end.hdg, r->end_heading))};
    fprintf(stderr, "%s %.1fx cadence %d: %.2fs to stop, peak %.3f m/s, end %.3fm %.3fdeg\n",
        r->gate, f, cadence, result.time, peak, result.end_error, result.heading_error);
    finish_fixture();
    return result;
}
