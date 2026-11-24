/* This fixture compiles as C++ to exercise the real SettingsWindow method. */
#include <cassert>
#include <cstdio>
#include "pushback_speed.h"

typedef bool bool_t;
typedef int bp_interface_mode_t;
#define B_FALSE false
#define B_TRUE true
#define UNUSED(x) (void)(x)
#define ASSERT(x) assert(x)
#define VERIFY(x) assert(x)
#define BP_WARN_LOG ""
#define BP_INTERFACE_MODE_CONFIG_KEY "interface_mode"
#define BP_NW_CORE_RELOAD 1
#define logMsg(...) ((void)0)

static bool bp_started, planner_open, pref_active, reload_rqst;
static bool interface_mode_change_rqst, legacy_magic_squares_reposition_rqst;
static bool inited = true;
static int requested_interface_mode, interface_mode;
static int bp_conf;
static void *reload_floop_ID = &bp_conf;
static unsigned writes, saves, schedules, invalidations, disables, enables;
static unsigned hides, repositions;
static bool get_pref_widget_status() { return pref_active; }
static void set_pref_widget_status(bool active) { pref_active = active; }
static bool apply_interface_mode(int) { return true; }
static bool bp_interface_mode_uses_legacy_magic_squares(int) { return true; }
static void main_intf_reposition() { repositions++; }
static void conf_set_i(int, const char *, int) { writes++; }
static void conf_set_b(int, const char *, bool) { writes++; }
static double saved_factor;
static void conf_set_d(int, const char *, double value)
{ saved_factor = value; writes++; }
static bool bp_conf_save() { saves++; return true; }
static void bp_nosewheel_status_invalidate(int) { invalidations++; }
static void bp_priv_disable() { disables++; }
static bool bp_priv_enable() { enables++; return true; }
static void XPLMScheduleFlightLoop(void *, int, int) { schedules++; }
static void bp_sched_reload(void);

class SettingsWindow {
public:
    bool save_disabled = false;
    bool legacy_routes = true;
    bool fast_ground_handling = true;
    int pushback_speed_selection = 0;
    void SetVisible(bool visible) { assert(!visible); hides++; }
    void SaveConfig(void);
};
static SettingsWindow window;
static SettingsWindow *setup_window = &window;

#include "reload_safety.inc"

static void reset()
{
    bp_started = planner_open = pref_active = reload_rqst = false;
    interface_mode_change_rqst = legacy_magic_squares_reposition_rqst = false;
    writes = saves = schedules = invalidations = disables = enables = 0;
    hides = repositions = 0;
    window.save_disabled = false;
    window.pushback_speed_selection = 0;
}

int main()
{
    for (unsigned mask = 0; mask < 8; mask++) {
        reset();
        bp_started = mask & 1;
        planner_open = mask & 2;
        window.save_disabled = mask & 4;
        assert(preferences_save_allowed(window.save_disabled) == (mask == 0));
        pref_active = true; /* Saving the current window is allowed when idle. */
        window.SaveConfig();
        if (mask) {
            assert(!writes && !saves && !schedules && !hides);
            assert(pref_active && !reload_rqst);
        } else {
            assert(writes == 3 && saves == 1 && schedules == 1 && hides == 1);
            assert(!pref_active && reload_rqst);
            bp_do_reload(0, 0, 0, NULL);
            assert(invalidations == 1 && disables == 1 && enables == 1);
            assert(!reload_rqst);
        }
    }
    reset();
    bp_conf_set_save_enabled(false);
    assert(window.save_disabled);
    bp_conf_set_save_enabled(true);
    assert(!window.save_disabled);
    setup_window = NULL;
    bp_conf_set_save_enabled(false);
    setup_window = &window;

    reset();
    planner_open = true;
    assert(!preferences_save_allowed(window.save_disabled));
    planner_open = false;
    assert(preferences_save_allowed(window.save_disabled));
    for (int choice = -1; choice <= 3; choice++) {
        reset();
        window.pushback_speed_selection = choice;
        assert(!writes && !saves); /* Staging and closing without Save do not write. */
        window.SaveConfig();
        assert(saved_factor == bp_pushback_speed_choice(choice));
        assert(writes == 3 && saves == 1);
    }

    for (unsigned mask = 1; mask < 4; mask++) {
        reset();
        bp_started = mask & 1;
        planner_open = mask & 2;
        bp_sched_reload();
        assert(!schedules && !reload_rqst);
    }
    for (unsigned mask = 1; mask < 8; mask++) {
        reset();
        bp_sched_reload();
        assert(reload_rqst && schedules == 1);
        bp_started = mask & 1;
        planner_open = mask & 2;
        pref_active = mask & 4;
        bp_do_reload(0, 0, 0, NULL);
        assert(!reload_rqst && !invalidations && !disables && !enables);
        bp_started = planner_open = pref_active = false;
        bp_do_reload(0, 0, 0, NULL);
        assert(!invalidations && !disables && !enables);
        bp_sched_reload();
        bp_do_reload(0, 0, 0, NULL);
        assert(invalidations == 1 && disables == 1 && enables == 1);
    }
    puts("Preferences save/reload safety: all cases passed");
}
