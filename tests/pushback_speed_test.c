/* SDK stubs surround actual extracted route, operation and steering functions. */
#include <assert.h>
#include <float.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef strlcpy
#undef strlcpy
#endif
#include <acfutils/helpers.h>
#include <acfutils/math.h>
#include "driving.h"
static int telemetry_enabled = 1;
#undef BP_ENABLE_RUNTIME_TELEMETRY
#define BP_ENABLE_RUNTIME_TELEMETRY telemetry_enabled
#include "telemetry.h"
#define BP_PLUGIN_VERSION "test"
#define TELEMETRY_SAMPLE_HZ 10
#define BP_ERROR_LOG ""

int bp_xp_ver = 12020;
#define MIN_STEERING_ARM_LEN 4
#define MAX_OFF_PATH_ANGLE 20
#define OFF_PATH_CORR_ANGLE 35
#define STEER_GATE(x, g) MIN(MAX((x), -(g)), (g))
#include "speed_constants.inc"
#define BP_WARN_LOG ""
#define BP_INFO_LOG ""
#define _(x) (x)

void XPLMDebugString(const char *s) { (void)s; }
void log_impl(const char *f, int l, const char *fmt, ...)
{
    va_list ap;
    fprintf(stderr, "%s:%d: ", f, l);
    va_start(ap, fmt); vfprintf(stderr, fmt, ap); va_end(ap);
    fputc('\n', stderr);
}
#include "speed_geometry.inc"
#include "speed_driving.inc"

typedef struct {
    double max_tow_fwd_speed, max_tow_rev_speed;
    double mass, max_fwd_speed, max_rev_speed, max_accel, max_decel, max_TE;
    const char *tug_name;
} tug_info_fixture_t;
typedef struct {
    const tug_info_fixture_t *info;
    vehicle_pos_t pos;
    vehicle_t veh;
    double cur_steer;
} tug_fixture_t;
static struct { tug_fixture_t *tug; } bp_ls;
static struct {
    vehicle_t veh;
    vehicle_pos_t cur_pos, d_pos, last_pos;
    list_t segs;
    vect2_t start_pos;
    double automatic_push_speed_factor, d_t, cur_t, reverse_t, last_mis_hdg;
    double last_force, last_t, step_start_t, start_hdg;
    int step;
    bool_t pause_hold;
    struct { int nw_i; } acf;
} bp;
static struct { bool_t active, requested; } push_manual;
static struct {
    bp_telemetry_t writer;
    drive_speed_limits_t published_speed_limits;
    double route_steer_cmd_deg, applied_steer_cmd_deg, nosewheel_steer_request_deg;
} bp_telem;
static struct { double effective_max_fwd_speed_mps, effective_max_rev_speed_mps; } metadata;
static struct {
    double lbrake, rbrake, axial_force, rot_force_N, tire_steer_cmd;
    double acf_mass, mtow;
    int landing_lights_on, taxi_light_on;
} drs;
static bool_t slave_mode, bp_started, late_plan_requested;
static bool_t parking_brake, near_end, can_start = B_TRUE;
static double last_speed, last_accel, last_tug_steer;
static unsigned push_calls, sound_calls, manual_calls, config_reads;
static bool_t last_decel;
static int config_storage, *bp_conf = &config_storage;
static double config_factor = 1;
static const char *config_text;
static bool_t config_readable = B_TRUE;
static double dr_getf(const double *v) { return *v; }
static void dr_setf(double *v, double x) { *v = x; }
static void dr_seti(int *v, int x) { *v = x; }
static void dr_setvf(double *v, const double *x, int i, int n)
{ (void)i; assert(n == 1); *v = *x; }
static bool_t pbrake_is_set(void) { return parking_brake; }
static void tug_set_TE_snd(tug_fixture_t *t, double s, double dt)
{ (void)t; (void)s; (void)dt; sound_calls++; }
static void tug_set_steering(tug_fixture_t *t, double s, double dt)
{ (void)t; (void)dt; last_tug_steer = s; }
static double tug_rear2acf_nw(void) { return 0.5; }
static void reorient_aircraft(double r, double p, double h)
{ assert(isfinite(r) && isfinite(p) && isfinite(h)); }
static void turn_nosewheel(double steer);
static void push_at_speed(double s, double a, bool_t snd, bool_t decel)
{ (void)snd; last_speed = s; last_accel = a; last_decel = decel; push_calls++; }
static vehicle_pos_t corr_acf_pos(void) { return bp.cur_pos; }
static bool_t nearing_end(void) { return near_end; }
static void telemetry_route_tracking(const seg_t *s, const vehicle_pos_t *p)
{ (void)s; (void)p; }
static bool_t bp_run_push_manual(void) { manual_calls++; return B_TRUE; }
typedef struct {
    int structSize, phase;
    float (*callbackFunc)(float, float, int, void *);
    void *refcon;
} XPLMCreateFlightLoop_t;
#define xplm_FlightLoop_Phase_BeforeFlightModel 1
static void *bp_floop;
static float bp_run(float a, float b, int c, void *d)
{ (void)a; (void)b; (void)c; (void)d; return 0; }
static bool_t bp_can_start(const char **reason)
{ *reason = "denied"; return can_start; }
void XPLMSpeakString(const char *s) { (void)s; }
static void *XPLMCreateFlightLoop(const XPLMCreateFlightLoop_t *f)
{ assert(f->callbackFunc == bp_run); return &config_storage; }
static void XPLMScheduleFlightLoop(void *f, int a, int b)
{ (void)f; (void)a; (void)b; }
static void bp_gather(void) {}
static bool_t bp_legacy_routes(void) { return B_FALSE; }
static bool_t emergency_tow_allows_persistent_routes(void) { return B_TRUE; }
static void route_save_legacy_stub(const list_t *s) { (void)s; }
#define route_save_legacy route_save_legacy_stub
static void nw_new_operation(bool_t b) { (void)b; }
static void bp_conf_set_save_enabled(bool_t b) { (void)b; }
static bool_t audio_sys_init(void) { return B_TRUE; }
static double bp_pushback_speed_factor(void);
static bool_t conf_get_str(int *conf, const char *key, const char **value)
{
    static char formatted[64];
    assert(conf && strcmp(key, "pushback_speed_factor") == 0);
    config_reads++;
    snprintf(formatted, sizeof(formatted), "%.17g", config_factor);
    if (config_readable) *value = config_text != NULL ? config_text : formatted;
    return config_readable;
}
#include "speed_controller.inc"

static char *bp_xpdir;
static char last_telemetry_path[2048];
static unsigned frame_resets;
static void telemetry_begin_frame(void) { frame_resets++; }
typedef int dr_t;
static void fdr_find(dr_t *dr, const char *key) { (void)key; *dr = 0; }
static void dr_gets(dr_t *dr, char *out, size_t n)
{ (void)dr; snprintf(out, n, "B738"); }
static void XPLMGetNthAircraftModel(int i, char *file, char *path)
{ (void)i; strcpy(file, "test.acf"); strcpy(path, "test/test.acf"); }
void lacf_strlcpy(char *restrict out, const char *restrict in, size_t n)
{ if (n) snprintf(out, n, "%s", in); }
char *mkpathname(const char *first, ...)
{
    if (first == bp_xpdir) return strdup(first);
    va_list ap; va_start(ap, first);
    const char *name = va_arg(ap, const char *); va_end(ap);
    snprintf(last_telemetry_path, sizeof(last_telemetry_path), "%s/%s", first, name);
    return strdup(last_telemetry_path);
}
bool_t file_exists(const char *path, bool_t *isdir)
{ struct stat st; if (stat(path, &st)) return B_FALSE;
  if (isdir) *isdir = S_ISDIR(st.st_mode); return B_TRUE; }
bool_t create_directory_recursive(const char *path)
{ (void)path; assert(!"Test directory must already exist"); return B_FALSE; }
#include "speed_publication.inc"
#include "speed_refresh.inc"

static vehicle_t vehicle(void)
{
    return (vehicle_t){.wheelbase = 10, .max_steer = 60,
        .max_fwd_spd = 4, .max_rev_spd = 1.11,
        .max_fwd_ang_vel = 6, .max_rev_ang_vel = 4,
        .max_centr_accel = 0.1, .max_accel = 0.25, .max_decel = 0.17};
}

static void clear_route(list_t *route)
{
    seg_t *s;
    while ((s = list_remove_head(route)) != NULL) free(s);
    list_destroy(route);
}

static void copy_route(list_t *route, const seg_t *segments, size_t n)
{
    list_create(route, sizeof(seg_t), offsetof(seg_t, node));
    for (size_t i = 0; i < n; i++) {
        seg_t *s = calloc(1, sizeof(*s)); assert(s);
        *s = segments[i]; memset(&s->node, 0, sizeof(s->node));
        list_insert_tail(route, s);
    }
}

static seg_t straight(bool_t back, double length)
{
    return (seg_t){.type = SEG_TYPE_STRAIGHT, .start_pos = {0, 0},
        .end_pos = {0, back ? -length : length}, .len = length, .backward = back};
}

static void same_double(double a, double b)
{ assert(memcmp(&a, &b, sizeof(a)) == 0); }

static void helper_tests(void)
{
    const double bad[] = {0, -1, 1.1, 3, NAN, INFINITY, -INFINITY, DBL_MAX};
    drive_speed_limits_t caps;
    for (size_t i = 0; i < sizeof(bad)/sizeof(*bad); i++) {
        assert(bp_pushback_speed_normalize(bad[i]) == 1);
        assert(!bp_pushback_speed_profile(bad[i], 4, 1.11, 8, 3, &caps));
    }
    for (int selection = -1; selection < 5; selection++) {
        double f = bp_pushback_speed_choice(selection);
        assert(f == (selection == 1 ? 1.5 : (selection == 2 ? 2 : 1)));
    }
    assert(!bp_pushback_speed_profile(2, 4, 1.11, 8, 3, NULL));
    assert(!bp_pushback_speed_profile(1, 4, 1.11, 8, 3, &caps));
    for (int which = 0; which < 4; which++) {
        const double invalid[] = {0, -1, NAN, INFINITY};
        for (size_t i = 0; i < 4; i++) {
            double v[] = {4, 1.11, 8, 3}; v[which] = invalid[i];
            assert(!bp_pushback_speed_profile(2, v[0], v[1], v[2], v[3], &caps));
        }
    }
    for (double f = 1.5; f <= 2; f += 0.5) {
        assert(bp_pushback_speed_profile(f, 4, 1.11, 8, 3, &caps));
        assert(caps.fwd_mps == 4 * f && caps.rev_mps == 1.11 * f);
        assert(bp_pushback_speed_profile(f, 4, 1.11, 0.7, 0.5, &caps));
        assert(fabs(caps.fwd_mps - 0.7) < 1e-15 && fabs(caps.rev_mps - 0.5) < 1e-15);
        assert(bp_pushback_speed_profile(f, 1.7778, 1.11, 1.7778, 1.7778, &caps));
        assert(fabs(caps.fwd_mps - 1.7778) < 1e-12);
        assert(fabs(caps.rev_mps - fmin(1.11 * f, 1.7778)) < 1e-12);
    }
    assert(bp_pushback_speed_profile(2, DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX, &caps));
    assert(isfinite(caps.fwd_mps) && caps.fwd_mps <= DBL_MAX);
    double unit = nextafter(0, 1);
    assert(bp_pushback_speed_profile(2, unit, unit, unit, unit, &caps));
    assert(caps.fwd_mps == unit && caps.rev_mps == unit);
    assert(bp_pushback_speed_profile(2, 2 * unit, 2 * unit, 3 * unit, 5 * unit, &caps));
    assert(caps.fwd_mps == 3 * unit && caps.rev_mps == 4 * unit);
    assert(bp_pushback_speed_tug_cap(20, 3, 2) == 3);
    assert(bp_pushback_speed_tug_cap(-20, 3, 2) == -2);
    assert(bp_pushback_speed_tug_cap(0, 3, 2) == 0);
    assert(bp_pushback_speed_tug_cap(NAN, 3, 2) == 0);
}

static unsigned parity_cases;
static void parity_tests(void)
{
    vehicle_t veh = vehicle(), saved = veh;
    for (int xp = 0; xp < 3; xp++) {
        bp_xp_ver = xp == 0 ? 10500 : (xp == 1 ? 11000 : 12020);
        for (int back = 0; back < 2; back++)
        for (int turn = 0; turn < 2; turn++)
        for (int right = 0; right < 2; right++)
        for (int chain = 0; chain < 5; chain++)
        for (int rear = 0; rear < 2; rear++)
        for (int sample = 0; sample <= 20; sample++) {
            seg_t segments[3] = {straight(back, 100), straight(back, 4), straight(back, 20)};
            if (turn) {
                segments[0].type = SEG_TYPE_TURN;
                segments[0].turn.r = right ? 20 : 80;
                segments[0].turn.right = right;
                segments[0].end_hdg = right ? 90 : 270;
                segments[0].end_pos = VECT2(right ? 20 : -80, 20);
            }
            if (chain >= 2) {
                segments[1].type = SEG_TYPE_TURN;
                segments[1].turn.r = 15;
                segments[1].turn.right = !right;
                segments[1].end_hdg = 45;
            }
            if (chain == 4) segments[1].backward = !back;
            size_t n = chain == 0 ? 1 : (chain == 1 ? 2 : 3);
            vehicle_pos_t pos = {.pos = {sample - 10, (back ? -1 : 1) * sample * 5.0},
                .hdg = sample * 12, .spd = (back ? -1 : 1) * (sample / 10.0)};
            veh.use_rear_pos = rear; veh.fixed_z_off = -4;
            list_t current, reference, nullable;
            copy_route(&current, segments, n); copy_route(&reference, segments, n);
            copy_route(&nullable, segments, n);
            double s1 = 123, v1 = 456, h1 = 0.5, s2 = s1, v2 = v1, h2 = h1;
            double s3 = s1, v3 = v1, h3 = h1;
            bool_t d1 = B_FALSE, d2 = d1, d3 = d1;
            bool_t r1 = drive_segs(&pos, &veh, &current, &h1, 0.02, &s1, &v1, &d1);
            bool_t r2 = reference_drive_segs(&pos, &veh, &reference, &h2, 0.02, &s2, &v2, &d2);
            bool_t r3 = drive_segs_with_speed_limits(&pos, &veh, &nullable, &h3,
                0.02, &s3, &v3, &d3, NULL);
            assert(r1 == r2 && r1 == r3 && d1 == d2 && d1 == d3);
            same_double(s1, s2); same_double(v1, v2); same_double(h1, h2);
            same_double(s1, s3); same_double(v1, v3); same_double(h1, h3);
            assert(list_count(&current) == list_count(&reference));
            assert(list_count(&current) == list_count(&nullable));
            const seg_t *a = list_head(&current), *b = list_head(&reference);
            for (; a && b; a = list_next(&current, a), b = list_next(&reference, b))
                assert(memcmp(a, b, offsetof(seg_t, node)) == 0);
            clear_route(&current); clear_route(&reference); clear_route(&nullable);
            parity_cases++;
        }
    }
    veh.use_rear_pos = saved.use_rear_pos; veh.fixed_z_off = saved.fixed_z_off;
    assert(memcmp(&veh, &saved, sizeof(veh)) == 0);
    bp_xp_ver = 12020;
}

static void planning_tests(void)
{
    vehicle_t veh = vehicle(), saved = veh;
    list_t route; copy_route(&route, NULL, 0);
    for (double f = 1.5; f <= 2; f += 0.5) {
        drive_speed_limits_t caps;
        assert(bp_pushback_speed_profile(f, 4, 1.11, 8, 3, &caps));
        for (int back = 0; back < 2; back++) {
            double cap = back ? caps.rev_mps : caps.fwd_mps;
            bool_t decel = B_FALSE;
            assert(straight_run_speed(&veh, &route, 1000, back, NULL, &decel, &caps) == cap);
            assert(!decel);
            double last = 0;
            for (int i = 0; i <= 1000; i++) {
                double d = i / 10.0;
                double v = straight_run_speed(&veh, &route, d, back, NULL, &decel, &caps);
                assert(v >= last && v <= cap && v >= 0.1);
                double braking_distance = d > 0.1 ? d - 0.1 : d;
                if (d >= 0.1) assert(v * v <= 0.01 + 2 * veh.max_decel * braking_distance + 1e-12);
                last = v;
            }
            assert(next_seg_speed(&veh, &route, NULL, back, &caps) == 0.1);
            seg_t opposite = straight(!back, 1000);
            assert(next_seg_speed(&veh, &route, &opposite, back, &caps) == 0.1);
            for (int radius = 5; radius <= 100; radius += 5) {
                double speed = turn_run_speed(&veh, &route, 90, radius, back, NULL, &decel, &caps);
                assert(speed <= cap && speed * speed / radius <= veh.max_centr_accel + 1e-12);
                assert(RAD2DEG(speed / radius) <= (back ? veh.max_rev_ang_vel : veh.max_fwd_ang_vel) + 1e-12);
            }
        }
        seg_t chain[] = {straight(B_FALSE, 0.2), straight(B_FALSE, 1000)};
        list_t lookahead; copy_route(&lookahead, chain, 2);
        assert(next_seg_speed(&veh, &lookahead, list_head(&lookahead), B_FALSE, &caps) == caps.fwd_mps);
        ((seg_t *)list_head(&lookahead))->type = SEG_TYPE_TURN;
        ((seg_t *)list_head(&lookahead))->turn.r = 10000;
        ((seg_t *)list_head(&lookahead))->end_hdg = 0.1;
        assert(next_seg_speed(&veh, &lookahead, list_head(&lookahead), B_FALSE, &caps) == caps.fwd_mps);
        clear_route(&lookahead);
        for (int side = -1; side <= 1; side += 2) {
            vehicle_pos_t off = {.pos = {side * 30, 0}, .hdg = side > 0 ? 60 : 300, .spd = 1};
            double h = 0, steer, speed;
            drive_on_line(&off, &veh, VECT2(0, 0), 0, caps.fwd_mps, 4, 2,
                B_TRUE, &h, 0.02, &steer, &speed);
            assert(fabs(speed) <= veh.max_rev_spd);
        }
    }
    assert(memcmp(&veh, &saved, sizeof(veh)) == 0);
    clear_route(&route);
}

static void increased_route_tests(void)
{
    vehicle_t veh = vehicle(), saved = veh;
    for (int fi = 1; fi <= 2; fi++)
    for (int back = 0; back < 2; back++)
    for (int right = 0; right < 2; right++)
    for (int sample = 1; sample < 9; sample++) {
        drive_speed_limits_t caps;
        assert(bp_pushback_speed_profile(bp_pushback_speed_choice(fi),
            veh.max_fwd_spd, veh.max_rev_spd, 8, 3, &caps));
        seg_t route[3] = {straight(back, 1000), straight(back, 1), straight(back, 1000)};
        double radius = right ? 20 : 80;
        double angle = sample * 10;
        route[0].type = SEG_TYPE_TURN;
        route[0].turn.r = radius; route[0].turn.right = right;
        route[0].end_hdg = right != back ? 90 : 270;
        route[0].end_pos = VECT2(right ? radius : -radius, back ? -radius : radius);
        route[1].type = SEG_TYPE_TURN; /* Same-side/S-turn recursive successor. */
        route[1].turn.r = radius;
        route[1].turn.right = !right;
        route[1].start_hdg = route[0].end_hdg;
        route[1].end_hdg = normalize_hdg(route[1].start_hdg + (right != back ? -90 : 90));
        vehicle_pos_t pos = {.pos = {right ? radius * (1 - cos(DEG2RAD(angle))) :
            -radius * (1 - cos(DEG2RAD(angle))), (back ? -1 : 1) * radius * sin(DEG2RAD(angle))},
            .hdg = right != back ? angle : 360 - angle, .spd = back ? -1 : 1};
        list_t enhanced, baseline;
        copy_route(&enhanced, route, 3); copy_route(&baseline, route, 3);
        double s1 = 0, s2 = 0, v1 = 0, v2 = 0, h1 = 0, h2 = 0;
        bool_t d1 = B_FALSE, d2 = B_FALSE;
        bool_t r1 = drive_segs_with_speed_limits(&pos, &veh, &enhanced, &h1,
            0.02, &s1, &v1, &d1, &caps);
        bool_t r2 = reference_drive_segs(&pos, &veh, &baseline, &h2,
            0.02, &s2, &v2, &d2);
        assert(r1 == r2 && r1 && isfinite(v1) && isfinite(s1));
        same_double(s1, s2); same_double(h1, h2);
        assert(fabs(v1) <= (back ? caps.rev_mps : caps.fwd_mps) + 1e-12);
        assert(fabs(s1) <= veh.max_steer);
        assert(list_count(&enhanced) == list_count(&baseline));
        assert(memcmp(&veh, &saved, sizeof(veh)) == 0);
        clear_route(&enhanced); clear_route(&baseline);
    }
}

static tug_info_fixture_t info = {.max_tow_fwd_speed = 8, .max_tow_rev_speed = 3,
    .tug_name = "test_tug"};
static tug_fixture_t tug;
static void reset_controller(double factor, bool_t back)
{
    memset(&bp, 0, sizeof(bp)); memset(&drs, 0, sizeof(drs));
    memset(&push_manual, 0, sizeof(push_manual)); memset(&tug, 0, sizeof(tug));
    bp.veh = vehicle(); bp.d_t = 0.02; bp.cur_t = 100;
    bp.automatic_push_speed_factor = factor;
    seg_t s = straight(back, 1000); copy_route(&bp.segs, &s, 1);
    tug.info = &info; tug.veh = vehicle(); tug.veh.wheelbase = 4;
    bp_ls.tug = &tug;
    parking_brake = near_end = slave_mode = B_FALSE;
    push_calls = sound_calls = manual_calls = 0;
    last_speed = last_accel = last_tug_steer = 0;
}

static void controller_tests(void)
{
    { /* Source checks verify independence from interface and Fast Handling. */
        for (int fi = 0; fi < 3; fi++) for (int back = 0; back < 2; back++)
        for (int angle = -85; angle <= 85; angle += 17) {
            double f = bp_pushback_speed_choice(fi);
            reset_controller(f, back);
            tug.pos.hdg = normalize_hdg(angle);
            assert(bp_run_push(B_FALSE) && push_calls == 1);
            double target = (back ? -1.11 : 4) * f / MAX(cos(DEG2RAD(angle)), 0.1);
            if (fi) target = bp_pushback_speed_tug_cap(target, info.max_tow_fwd_speed, info.max_tow_rev_speed);
            assert(fabs(last_speed - target) < 1e-12 && last_accel == bp.veh.max_accel);
            assert(!last_decel);
            speed_metadata();
            assert(metadata.effective_max_fwd_speed_mps == 4 * f);
            assert(metadata.effective_max_rev_speed_mps == 1.11 * f);
            clear_route(&bp.segs);
        }
    }
    for (int gate = 0; gate < 7; gate++) {
        reset_controller(2, B_TRUE);
        if (gate == 0) drs.lbrake = BRAKE_PEDAL_THRESH;
        if (gate == 1) drs.rbrake = BRAKE_PEDAL_THRESH;
        if (gate == 2) parking_brake = B_TRUE;
        if (gate == 3) bp.reverse_t = bp.cur_t - 0.5;
        if (gate == 4) bp.pause_hold = B_TRUE;
        if (gate == 5) push_manual.active = B_TRUE;
        assert(bp_run_push(gate == 4 || gate == 6));
        if (gate < 3) assert(!push_calls && sound_calls == 1 && bp.last_force == 0);
        if (gate == 3 || gate == 4 || gate == 6) assert(push_calls == 1 && last_speed == 0);
        if (gate == 5) assert(!push_calls && manual_calls == 1);
        assert(bp.automatic_push_speed_factor == 2);
        clear_route(&bp.segs);
    }
    reset_controller(2, B_FALSE);
    drive_speed_limits_t caps;
    slave_mode = B_TRUE; assert(!automatic_push_speed_limits(&caps)); speed_metadata();
    assert(metadata.effective_max_fwd_speed_mps == 4);
    slave_mode = B_FALSE; push_manual.requested = B_TRUE;
    assert(!automatic_push_speed_limits(&caps)); speed_metadata();
    assert(metadata.effective_max_rev_speed_mps == 1.11);
    push_manual.requested = B_FALSE; bp_ls.tug = NULL;
    assert(!automatic_push_speed_limits(&caps)); bp_ls.tug = &tug;
    info.max_tow_fwd_speed = NAN;
    assert(bp_run_push(B_FALSE)); assert(bp.automatic_push_speed_factor == 1 && last_speed == 4);
    info.max_tow_fwd_speed = 8;
    assert(bp_run_push(B_FALSE) && last_speed == 4); /* No silent re-enable. */
    clear_route(&bp.segs);
    reset_controller(2, B_FALSE);
    info.max_tow_fwd_speed = 0.7; info.max_tow_rev_speed = 0.5;
    assert(bp_run_push(B_FALSE) && fabs(last_speed - 0.7) < 1e-12);
    speed_metadata(); assert(fabs(metadata.effective_max_fwd_speed_mps - 0.7) < 1e-12);
    clear_route(&bp.segs); info.max_tow_fwd_speed = 8; info.max_tow_rev_speed = 3;

    reset_controller(2, B_TRUE);
    near_end = B_TRUE;
    assert(bp_run_push(B_FALSE) && bp_telem.applied_steer_cmd_deg == 0);
    clear_route(&bp.segs);
    reset_controller(2, B_FALSE);
    clear_route(&bp.segs);
    seg_t reversal[] = {straight(B_FALSE, 100), straight(B_TRUE, 100)};
    reversal[1].start_pos.y = 100; reversal[1].end_pos.y = 0;
    copy_route(&bp.segs, reversal, 2); bp.cur_pos.pos.y = 100;
    assert(bp_run_push(B_FALSE) && list_count(&bp.segs) == 1);
    assert(bp.reverse_t == bp.cur_t && last_speed == 0 && push_calls == 1);
    double reversal_start = bp.cur_t;
    bp.cur_t = reversal_start + 2 * STATE_TRANS_DELAY - 0.01;
    assert(bp_run_push(B_FALSE) && last_speed == 0 && bp.reverse_t == reversal_start);
    bp.cur_t = reversal_start + 2 * STATE_TRANS_DELAY;
    assert(bp_run_push(B_FALSE) && bp.reverse_t == 0 && last_speed < 0);
    bp.reverse_t = reversal_start;
    bp.cur_t = reversal_start + 2 * STATE_TRANS_DELAY + 0.01;
    assert(bp_run_push(B_FALSE) && bp.reverse_t == 0 && last_speed < 0);
    bp.cur_pos.pos.y = -1;
    assert(!bp_run_push(B_FALSE) && list_count(&bp.segs) == 0);
    clear_route(&bp.segs);
}

static void lifecycle_tests(void)
{
    const double values[] = {1, 1.5, 2, 0, -1, 1.1, NAN, INFINITY};
    for (size_t i = 0; i < sizeof(values)/sizeof(*values); i++) {
        config_factor = values[i]; config_readable = B_TRUE;
        assert(bp_pushback_speed_factor() == bp_pushback_speed_normalize(values[i]));
    }
    bp_conf = NULL; assert(bp_pushback_speed_factor() == 1); bp_conf = &config_storage;
    config_readable = B_FALSE; assert(bp_pushback_speed_factor() == 1);
    config_readable = B_TRUE;
    const char *invalid[] = {"", " ", "2x", "1.5garbage", "2 1", "nonsense",
        "nan", "inf", "1e9999", "0x1p9999", "-2", "1.5001"};
    for (size_t i = 0; i < sizeof(invalid)/sizeof(*invalid); i++) {
        config_text = invalid[i]; assert(bp_pushback_speed_factor() == 1);
    }
    config_text = " 1.5 \t"; assert(bp_pushback_speed_factor() == 1.5);
    config_text = "2.00000000000000000"; assert(bp_pushback_speed_factor() == 2);
    config_text = NULL;
    reset_controller(1, B_TRUE); bp_started = B_FALSE; can_start = B_FALSE;
    config_factor = 2; config_reads = 0;
    assert(!bp_start() && config_reads == 0 && bp.automatic_push_speed_factor == 1);
    can_start = B_TRUE;
    assert(bp_start() && config_reads == 1 && bp.automatic_push_speed_factor == 2);
    config_factor = 1.5;
    assert(bp_start() && config_reads == 1 && bp.automatic_push_speed_factor == 2);
    assert(bp_run_push(B_FALSE) && last_speed == -2.22);
    bp_started = B_FALSE;
    assert(bp_start() && config_reads == 2 && bp.automatic_push_speed_factor == 1.5);
    clear_route(&bp.segs);
}

static void check_emitted_limits(double expected_fwd, double expected_rev)
{
    FILE *file = fopen(last_telemetry_path, "r"); assert(file);
    char line[2048]; double fwd = -1, rev = -1;
    while (fgets(line, sizeof(line), file)) {
        (void)sscanf(line, "# effective_max_fwd_speed_mps,%lf", &fwd);
        (void)sscanf(line, "# effective_max_rev_speed_mps,%lf", &rev);
    }
    fclose(file);
    assert(fabs(fwd - expected_fwd) < 1e-9 && fabs(rev - expected_rev) < 1e-9);
}

static void telemetry_transition_tests(void)
{
    reset_controller(2, B_FALSE);
    telemetry_start(); assert(bp_telem.writer.fp);
    check_emitted_limits(8, 2.22);
    unsigned resets = frame_resets;
    char previous[2048]; strcpy(previous, last_telemetry_path);
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    push_manual.requested = B_TRUE; /* Connect-first -> pending Manual start. */
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) != 0);
    check_emitted_limits(4, 1.11); strcpy(previous, last_telemetry_path);
    push_manual.active = B_TRUE; push_manual.requested = B_FALSE;
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    push_manual.active = B_FALSE; /* Automatic movement after reconnect. */
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) != 0);
    check_emitted_limits(8, 2.22); strcpy(previous, last_telemetry_path);
    assert(frame_resets == resets);
    telemetry_enabled = 0; push_manual.active = B_TRUE;
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    telemetry_enabled = 1;
    bp_telemetry_close(&bp_telem.writer);
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    clear_route(&bp.segs);
}

static void reverse_steering_baseline_test(void)
{
    for (int back = 0; back < 2; back++) for (int side = -1; side <= 1; side += 2) {
        reset_controller(2, back);
        tug.pos.spd = back ? -2.22 : 8;
        turn_nosewheel(side * 20);
        /* Record, do not fix, the baseline signed reverse damping asymmetry. */
        if (back) assert(fabs(fabs(last_tug_steer) - 60) < 1e-8);
        else assert(fabs(last_tug_steer) < 60);
        assert(isfinite(drs.tire_steer_cmd));
        clear_route(&bp.segs);
    }
}

int main(int argc, char **argv)
{
    assert(argc == 2); bp_xpdir = argv[1];
    helper_tests(); parity_tests(); planning_tests(); increased_route_tests(); controller_tests();
    lifecycle_tests(); telemetry_transition_tests(); reverse_steering_baseline_test();
    printf("Pushback speed: %u bit-identical legacy route cases; helper, lookahead, "
        "controller, metadata, lifecycle and steering baseline cases passed\n", parity_cases);
    return 0;
}
