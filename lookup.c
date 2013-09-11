#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "sensor_IF.h"

#include "lookup.h"
#include "lookup_IF.h"

extern  LOOKUP_STAT gkLookupStat;

/* ÉvÉçÉgÉ^ÉCÉvêÈåæ */
void    lookup_start(void);
void    lookup_tail_down(void);
void    lookup_cock(void);
void    lookup_tail_up(void);
void    lookup_forward(void);
void    lookup_standing(void);

void    lookup_stand(F32 foward, F32 turn, F32 gyro_offset);

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_ini(
            void
        )
{
    gkLookupStat.lookupStat = LOOKUP_STAT_INI;

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_entry(
            void
        )
{
    switch (gkLookupStat.lookupStat) {
        case LOOKUP_STAT_INI:
            lookup_start();
            break;
        case LOOKUP_STAT_TAIL_DOWN:
            lookup_tail_down();
            break;
        case LOOKUP_STAT_COCK:
            lookup_cock();
            break;
        case LOOKUP_STAT_TAIL_UP:
            lookup_tail_up();
            break;
        case LOOKUP_STAT_FORWARD:
            lookup_forward();
            break;
        case LOOKUP_STAT_STAND:
            lookup_standing();
            break;
        case LOOKUP_STAT_RUN:
            lookup_stand((F32)10, (F32)0, GYRO_OFFSET);
            break;
        default:
            break;
    }

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_start(
            void
        )
{
    U16 lightness;
    F32 turn;
    F32 forward = 0;
    int sonar;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    if (lightness > (U16)530) {
        turn = -50;
    } else {
        turn = 50;
    }

    sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
    if (sonar < 25) {
        forward = -10;
    } else { /* nothing */ }

    lookup_stand(forward, turn, GYRO_OFFSET);

    nxt_motor_set_count(PORT_MOTOR_TAIL, 0);
    nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 1);
//    gkLookupStat.lookupStat = LOOKUP_STAT_TAIL_DOWN;

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_tail_down(
            void
        )
{
    int tail;

    lookup_stand((F32)0, (F32)0, GYRO_OFFSET);

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < LOOKUP_COCK_START_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)10, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 1);
        gkLookupStat.lookupStat = LOOKUP_STAT_COCK;
    }

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_cock(
            void
        )
{
    int tail;
    static int cnt = 0;

    lookup_stand((F32)0, (F32)0, GYRO_OFFSET - 10);

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < LOOKUP_COCK_START_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)10, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-10, 1);
    }

    if (cnt >= 125) {
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        gkLookupStat.lookupStat = LOOKUP_STAT_TAIL_UP;
    } else {
        cnt++;
    }

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_tail_up(
            void
        )
{
    int tail;
    static int target   = LOOKUP_COCK_START_ANGLE;
    static int cnt      = 0;

    if (cnt == 50) {
        target -= 5;
        cnt = 0;
    } else {
        cnt++;
    }

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < target) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)30, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-30, 1);
    }

    if (target <= LOOKUP_COCK_ANGLE) {
        nxt_motor_set_count(PORT_MOTOR_L, 0);
        nxt_motor_set_count(PORT_MOTOR_R, 0);
        gkLookupStat.lookupStat = LOOKUP_STAT_FORWARD;
    } else { /* nothing */ }

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_forward(
            void
        )
{
    int tail;
    int right;

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < LOOKUP_COCK_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)80, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-80, 1);
    }

    nxt_motor_set_speed(PORT_MOTOR_L, 30, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, 30, 1);

    right = nxt_motor_get_count(PORT_MOTOR_R);
    if (right > 720) {
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        nxt_motor_set_count(PORT_MOTOR_TAIL, 0);
        gkLookupStat.lookupStat = LOOKUP_STAT_STAND;
    } else { /* nothing */ }

    return;
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_standing(
            void
        )
{
    int tail;
    static int cnt;

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < 23) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)68, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 1);
    }

    cnt++;
    if (cnt > 250) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 0);
        balance_init();
        nxt_motor_set_count(PORT_MOTOR_L, 0);
        nxt_motor_set_count(PORT_MOTOR_R, 0);
        gkLookupStat.lookupStat = LOOKUP_STAT_RUN;
    } else { /* nothing */ }
}

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
**********************************************************/
void    lookup_stand(
            F32 foward,
            F32 turn,
            F32 gyro_offset
        )
{
    S8  pwm_l;
    S8  pwm_r;

    /* ì|óßêßå‰ */
    balance_control(
        (F32)foward,
        (F32)turn,
        (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
        gyro_offset,
        (F32)nxt_motor_get_count(PORT_MOTOR_L),
        (F32)nxt_motor_get_count(PORT_MOTOR_R),
        (F32)ecrobot_get_battery_voltage(),
        &pwm_l,
        &pwm_r
    );

    nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

    return;
}
