#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#define B_FALSE 0
#define ASSERT assert
#define MAX(a,b) fmax(a,b)
#define MIN(a,b) fmin(a,b)
#define ABS fabs
#define DEG2RAD(x) ((x)*0.017453292519943295)
#define RAD2DEG(x) ((x)*57.29577951308232)
#define UNUSED(x) ((void)(x))
typedef int bool_t;
typedef struct { double x,y; } vect2_t;
#define VECT2(x,y) ((vect2_t){(x),(y)})
static vect2_t vect2_add(vect2_t a,vect2_t b) { return VECT2(a.x+b.x,a.y+b.y); }
static vect2_t vect2_neg(vect2_t a) { return VECT2(-a.x,-a.y); }
static vect2_t vect2_scmul(vect2_t a,double b) { return VECT2(a.x*b,a.y*b); }
static vect2_t hdg2dir(double h) { return VECT2(sin(DEG2RAD(h)),cos(DEG2RAD(h))); }
static vect2_t vect2_rot(vect2_t a,double h) {
    double c=cos(DEG2RAD(h)),s=sin(DEG2RAD(h));
    return VECT2(a.x*c+a.y*s,-a.x*s+a.y*c);
}
static double normalize_hdg(double h) { return fmod(h+360,360); }
typedef struct { double max_fwd_spd,max_rev_spd,max_steer,wheelbase; } veh_t;
typedef struct { int anim_debug,num_fwd_gears,num_rev_gears;
    double rear_z,max_fwd_speed,max_rev_speed; } info_t;
typedef struct { vect2_t pos; double spd,hdg; } pose_t;
typedef struct { int segs,steer_override,TE_override,load_in_prog;
    double last_mis_hdg,cur_steer; veh_t veh,veh_slow; pose_t pos; info_t *info;
} tug_t;
static void *list_head(int *s) { return *s ? s : NULL; }
static void drive_segs(pose_t *p,veh_t *v,int *s,double *m,double dt,
    double *st,double *sp,void *d) {
    (void)p;(void)v;(void)s;(void)m;(void)dt;(void)st;(void)sp;(void)d;
    assert(0 && "coupled tug must not have driving segments");
}
static double sound_dt;
static void tug_set_TE_snd(tug_t *t,double te,double dt) {
    (void)t;(void)te;sound_dt=dt;
}
#include "steps.inc"

int main(void) {
    info_t info={.num_fwd_gears=3,.num_rev_gears=2,.rear_z=1,
                 .max_fwd_speed=4,.max_rev_speed=3};
    const double factors[]={1,1.5,2}, fps[]={20,30,60,120};
    int cases=0;
    for (int f=0;f<3;f++) for (int k=0;k<4;k++)
    for(int sign=-1;sign<=1;sign+=2) for(int angle=-30;angle<=30;angle+=30)
    for(int override=0;override<=1;override++) {
        double F=factors[f],dt=1/fps[k];
        tug_t ref={.veh={4,3,65,3},.cur_steer=angle,.steer_override=override,
                   .pos={{1,2},1.8*sign,88},.info=&info};
        tug_t fast=ref;
        for(int j=0;j<6;j++) {
            double feedback=sign*(1.8-0.05*j);
            ref.pos.spd=feedback;
            reference_run(&ref,dt*F,0);
            if(F==1) {
                fast.pos.spd=feedback;
                tug_run(&fast,dt,0);
            } else {
                fast.pos.spd=99; /* draw callback is not authoritative */
                tug_run_attached(&fast,dt,F,feedback*F);
            }
            assert(sound_dt==dt);
            assert(fabs(ref.pos.hdg-fast.pos.hdg)<1e-10);
            assert(fabs(ref.pos.pos.x-fast.pos.pos.x)<1e-10);
            assert(fabs(ref.pos.pos.y-fast.pos.pos.y)<1e-10);
            assert(fabs(ref.cur_steer-fast.cur_steer)<1e-10);
            assert(fabs(ref.pos.spd*F-fast.pos.spd)<1e-10);
        }
        cases++;
    }
    printf("%d coupled tug reference cases passed\n",cases);
    return 0;
}
