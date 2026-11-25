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

#if FROZEN
int bp_xp_ver = 12020;
#else
extern int bp_xp_ver;
int route_envelope_cases(void);
#endif
#define MIN_STEERING_ARM_LEN 4
#define MAX_OFF_PATH_ANGLE 20
#define OFF_PATH_CORR_ANGLE 35
#define STEER_GATE(x, g) MIN(MAX((x), -(g)), (g))
#include "speed_constants.inc"
#include "pushback_speed.h"
#include "pushback_step.h"
#include "clear_signal_gate.h"
#include "fast_brake_handoff.h"
#include "handling_timing.h"
#include <stdbool.h>
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


typedef int dr_t;
typedef void wav_t;
typedef void *XPLMObjectRef, *XPLMInstanceRef;
#if FROZEN
#define CONFIG_KEY "pushback_speed_factor"
#else
#define CONFIG_KEY "pushback_time_factor"
#endif
#include "speed_fixture.inc"
static struct { tug_t *tug; void *wing_walker; } bp_ls;
static bp_state_t bp;
static push_manual_t push_manual;
static bool_t bp_started, slave_mode, late_plan_requested, coupled;
static bool_t op_complete, bp_connected, tug_pending_mode, tug_auto_start, tug_starts_next_plane;
static bool_t cfg_ignore_park_break, parking_brake, can_start = B_TRUE, fast_handling;
static const char *bp_hint_status_str;
static unsigned brake_writes, quaternion_writes, manual_calls, config_reads;
static double last_d_hdg;
static int config_storage, *bp_conf = &config_storage;
static double config_factor = 1;
static const char *config_text;
static bool_t config_readable = B_TRUE;
static double dr_getf(const double *v) { return *v; }
static int dr_geti(const double *v) { return (int)*v; }
static void dr_setf(double *v, double x) { assert(isfinite(x)); *v = x; }
static void dr_seti(double *v, int x) { *v = x; }
static int dr_getvf(const double *v, double *x, int i, int n)
{ (void)i; assert(n == 1); *x = *v; return n; }
static int dr_getvi(const double *v, int *x, int i, int n)
{ (void)i; assert(n == 1); *x = (int)*v; return n; }
static void dr_setvf(double *v, const double *x, int i, int n)
{
    if (v == &drs.quaternion) {
        assert(i == 0 && n == 4);
        for (int j = 0; j < 4; j++) assert(isfinite(x[j]));
        /* The simulator consumes a float quaternion, not the requested yaw
         * delta. At zero roll/pitch recover the emitted yaw directly. */
        double h = normalize_hdg(RAD2DEG(2 * atan2((float)x[3], (float)x[0])));
        last_d_hdg = rel_hdg(normalize_hdg(drs.hdg), h);
        quaternion_writes++;
        return;
    }
    (void)i; assert(n == 1 && isfinite(*x)); *v = (float)*x;
}
static bool_t pbrake_is_set(void) { return parking_brake; }
static void brakes_set(bool_t set)
{ brake_writes++; drs.lbrake = drs.rbrake = set ? 1 : 0; }
static void tug_set_TE_snd(tug_t *t, double s, double dt)
{ (void)t; assert(isfinite(s) && dt == bp.d_t); }
static void tug_set_TE_override(tug_t *t, bool_t b) { t->TE_override = b; }
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

static float bp_run(float, float, int, void *);
static bool_t bp_can_start(const char **reason)
{ *reason = "denied"; return can_start; }
void XPLMSpeakString(const char *s) { (void)s; }
static void *XPLMCreateFlightLoop(const XPLMCreateFlightLoop_t *f)
{ assert(f->callbackFunc == bp_run); return &config_storage; }
static void XPLMScheduleFlightLoop(void *f, int a, int b)
{ (void)f; (void)a; (void)b; }
static bool_t bp_legacy_routes(void) { return B_FALSE; }
static bool_t emergency_tow_allows_persistent_routes(void) { return B_TRUE; }
static void route_save_legacy_stub(const list_t *s) { (void)s; }
#define route_save_legacy route_save_legacy_stub
static void nw_new_operation(bool_t b) { (void)b; }
static void bp_conf_set_save_enabled(bool_t b) { (void)b; }
static bool_t audio_sys_init(void) { return B_TRUE; }

static bool_t conf_get_str(int *conf, const char *key, const char **value)
{
    static char formatted[64];
    assert(conf && strcmp(key, CONFIG_KEY) == 0);
    config_reads++;
    snprintf(formatted, sizeof(formatted), "%.17g", config_factor);
    if (config_readable) *value = config_text != NULL ? config_text : formatted;
    return config_readable;
}


static char *bp_xpdir;
static char last_telemetry_path[2048];

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
{
    struct stat st;
    if (stat(path, &st))
        return B_FALSE;
    if (isdir)
        *isdir = S_ISDIR(st.st_mode);
    return B_TRUE;
}
bool_t create_directory_recursive(const char *path)
{ (void)path; assert(!"Test directory must already exist"); return B_FALSE; }


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
            list_t current, reference;
            copy_route(&current, segments, n); copy_route(&reference, segments, n);

            double s1 = 123, v1 = 456, h1 = 0.5, s2 = s1, v2 = v1, h2 = h1;

            bool_t d1 = B_FALSE, d2 = d1;
            bool_t r1 = drive_segs(&pos, &veh, &current, &h1, 0.02, &s1, &v1, &d1);
            bool_t r2 = reference_drive_segs(&pos, &veh, &reference, &h2, 0.02, &s2, &v2, &d2);
            assert(r1 == r2 && d1 == d2);
            same_double(s1, s2); same_double(v1, v2); same_double(h1, h2);

            assert(list_count(&current) == list_count(&reference));

            const seg_t *a = list_head(&current), *b = list_head(&reference);
            for (; a && b; a = list_next(&current, a), b = list_next(&reference, b))
                assert(memcmp(a, b, offsetof(seg_t, node)) == 0);
            clear_route(&current); clear_route(&reference);
            parity_cases++;
        }
    }
    veh.use_rear_pos = saved.use_rear_pos; veh.fixed_z_off = saved.fixed_z_off;
    assert(memcmp(&veh, &saved, sizeof(veh)) == 0);
    bp_xp_ver = 12020;
}


#include "motion_lifecycle_fixture.h"
#include "speed_controller.inc"
#include "motion_lifecycle_test.h"
#if !FROZEN
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
    reset_controller(2, B_FALSE, 1000);
    arm_push();
    telemetry_start(); assert(bp_telem.writer.fp);
    check_emitted_limits(8, 2.22);

    char previous[2048]; strcpy(previous, last_telemetry_path);
    telemetry_begin_frame(); telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    push_manual.requested = B_TRUE; /* Connect-first -> pending Manual start. */
    telemetry_begin_frame();
    bp_telem.applied_steer_cmd_deg = 17;
    telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) != 0);
    assert(bp_telem.applied_steer_cmd_deg == 17); /* rotation must preserve this frame */
    check_emitted_limits(4, 1.11); strcpy(previous, last_telemetry_path);
    push_manual.active = B_TRUE; push_manual.requested = B_FALSE;
    telemetry_begin_frame(); telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    push_manual.active = B_FALSE; /* Automatic movement after reconnect. */
    telemetry_begin_frame(); telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) != 0);
    check_emitted_limits(8, 2.22); strcpy(previous, last_telemetry_path);
    assert(isnan(bp_telem.applied_steer_cmd_deg));
    telemetry_enabled = 0; push_manual.active = B_TRUE;
    telemetry_begin_frame(); telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    telemetry_enabled = 1;
    bp_telemetry_close(&bp_telem.writer);
    telemetry_begin_frame(); telemetry_refresh_speed_limits(); assert(strcmp(previous, last_telemetry_path) == 0);
    finish_fixture();
}


#endif
int main(int argc, char **argv)
{
    assert(argc == 4); bp_xpdir = argv[1];
    parity_tests();
    reference_traces(argv[2]);
#if !FROZEN
    assert(route_envelope_cases() == 0);
    lifecycle_tests();
    telemetry_transition_tests();
#endif
    closed_loop_tests(argv[2], argv[3]);
    printf("Verified %u frozen driving cases\n", parity_cases);
    return 0;
}
