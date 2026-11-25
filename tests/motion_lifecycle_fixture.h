/* SDK boundaries for the real bp_run dispatcher. A transition into an
 * unmodelled handling step must fail; it is never a successful fast finish. */
enum { RWY_FRICTION_GOOD, RWY_FRICTION_MED, RWY_FRICTION_POOR };
enum { MSG_RDY2CONN_NOPARK, MSG_RDY2CONN, MSG_OP_COMPLETE };
static bool_t plan_complete;
static struct { bool_t custody; } nw_status;
static double anim_speed, anim_heading, draw_heading;
static unsigned draws, animations, complete_messages;

static void tug_draw(tug_t *t, double now)
{ assert(now == bp.cur_t); draw_heading = t->pos.hdg; draws++; }
static void tug_anim(tug_t *t, double dt, double now)
{
    assert(dt == bp.d_t && now == bp.cur_t);
    anim_speed = t->pos.spd; anim_heading = t->pos.hdg; animations++;
}
static void msg_play(int id)
{ assert(id == MSG_OP_COMPLETE); complete_messages++; }
#define nw_fault(reason) ((void)0)
#define nw_publish() ((void)0)
#define nw_observe_step() ((void)0)
#define nw_end_window() ((void)0)
#define nw_end_requested() ((void)0)
#define nw_tug_basis() (0)
#define nw_rate_written(p) ((void)0)
#define nw_winch_geometry(a,b,c) (false)
#define nw_released() ((void)0)
#define bp_reconnect_notify() ((void)0)
#define telemetry_record(b) ((void)(b))
#define disco_intf_hide() ((void)0)
#define manual_bp_stop() ((void)0)
#define bp_coupling_active() (coupled)
#define acf_plg_debut() ((void)0)
#define acf_plg_fini() ((void)0)
#define artificial_delay(t) bp_handling_duration((t), fast_handling)
#define handling_fraction(t,d) bp_handling_fraction((t),(d),fast_handling)
#define wing_walker_update(...) abort()
#define tug_set_lift_pos(x) ((void)(x), abort())
#define tug_set_lift_arm_pos(...) abort()
#define tug_set_lift_in_transit(...) abort()
#define tug_set_cradle_beeper_on(...) abort()
#define tug_set_tire_sense_pos(...) abort()
#define acf_doors_closed(b) (false)
#define msg_stop() abort()
#define bp_complete() abort()
#define pb_enter_ungrabbing(...) abort()
#define pb_step_tug_load() (abort(), false)
#define disable_replanning() abort()
#define enable_replanning() abort()
#define prop_single_adjust() abort()
#define pb_step_start() abort()
#define pb_step_driving_up_close() abort()
#define pb_step_waiting_for_doors() abort()
#define pb_step_waiting_for_pbrake() abort()
#define pb_step_driving_up_connect() abort()
#define pb_step_grab() abort()
#define pb_step_lift() abort()
#define pb_step_connected() abort()
#define pb_step_stopped() abort()
#define pb_step_lowering() abort()
#define pb_step_ungrabbing() abort()
#define pb_step_waiting4ok2disco() abort()
#define pb_step_closing_cradle() abort()
#define pb_step_starting2clear() abort()
#define pb_step_clear_signal() abort()
