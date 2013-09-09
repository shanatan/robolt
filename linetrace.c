#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "sensor_IF.h"

#include "main.h"
#include "linetrace.h"
#include "linetrace_IF.h"

extern  ROBOT_STAT  gkRobotStat;

/* ÉvÉçÉgÉ^ÉCÉvêÈåæ */
static  void    lt_calc_pid(U16 light, F32 *pturn);

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
void    lt_pol_triger(
            void
        )
{
    U8 currentStatus = ecrobot_get_touch_sensor(PORT_TOUCH);
    S8  pwm_l;
    S8  pwm_r;

    if (currentStatus == 1) {
        /* ì|óßêßå‰ */
        balance_control(
            (F32)0,
            (F32)0,
            (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
            GYRO_OFFSET,
            (F32)nxt_motor_get_count(PORT_MOTOR_L),
            (F32)nxt_motor_get_count(PORT_MOTOR_R),
            (F32)ecrobot_get_battery_voltage(),
            &pwm_l,
            &pwm_r);
        nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
        nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

        nxt_motor_set_count(PORT_MOTOR_TAIL, (int)0);

        gkRobotStat.robotStat = ROBOT_STAT_STAND;
    } else { /* NOTHING */ }
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
void    lt_statnd(
            void
        )
{
    S8  pwm_l;
    S8  pwm_r;
    int tail;

    /* ì|óßêßå‰ */
    balance_control(
        (F32)0,
        (F32)0,
        (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
        GYRO_OFFSET,
        (F32)nxt_motor_get_count(PORT_MOTOR_L),
        (F32)nxt_motor_get_count(PORT_MOTOR_R),
        (F32)ecrobot_get_battery_voltage(),
        &pwm_l,
        &pwm_r);

    nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail < -40) {
        gkRobotStat.robotStat = ROBOT_STAT_TRACE;
        //gkRobotStat.robotStat = ROBOT_STAT_LOOKUP;
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-50, 1);
    }
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
void    lt_linetrace(
            void
        )
{
    S8  pwm_l;
    S8  pwm_r;
    F32 forward;
    F32 turn;
    F32 u_turn;
    U16 lightness;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    lt_calc_pid(lightness, &turn);

    if (turn > 100) {
        turn = 100;
    } else if (turn < -100) {
        turn = -100;
    }

    /* ê‚ëŒílÇÇ∆ÇÈ */
    if (turn > 0) {
        u_turn = turn;
    } else {
        u_turn = (-1) * turn;
    }

    /* ê˘âÒó Ç©ÇÁëOêió Çí≤êÆÇ∑ÇÈ */
    forward = 160 - (u_turn / 1.0);

    balance_control(
        (F32)forward,
        turn,
        (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
        GYRO_OFFSET,
        (F32)nxt_motor_get_count(PORT_MOTOR_L),
        (F32)nxt_motor_get_count(PORT_MOTOR_R),
        (F32)ecrobot_get_battery_voltage(),
        &pwm_l,
        &pwm_r);

    nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
    nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

    display_clear(0);
    display_goto_xy(0, 0);
    display_int(turn, 5);
    display_goto_xy(0, 1);
    display_int(forward, 5);
    display_update();

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
static  void    lt_calc_pid(
                    U16 light,
                    F32 *pturn
                )
{
    static S32  diff[2];
    static F32  integral;
    static U32  cnt;

    F32 p;
    F32 i;
    F32 d;

    /* 1ïbÇ≤Ç∆Ç…êœéZÇÉNÉäÉAÇ∑ÇÈ */
    cnt++;
    if (cnt > (INTEGRAL_CLEAR_TIME / (U32)4)) {
        integral = (F32)0;
    } else { /* nothing */ }

    diff[0] = diff[1];
    diff[1] = light - BLACK_WHITE_THRESHOLD;
    integral += (diff[1] + diff[0]) / 2.0 * 0.004;

    p = 1.75 * diff[1];
    i = 0.35 * integral;
    d = 0.45 * (diff[1] - diff[0]) / 0.004;

    //*pturn = ((light - BLACK_WHITE_THRESHOLD) * -1.2);

    *pturn = (-1) * (p + i + d);
}
