#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "sensor_IF.h"

#include "main.h"
#include "linetrace_IF.h"

#define BLACK_WHITE_THRESHOLD   (U16)530

extern  ROBOT_STAT  gkRobotStat;

S32 diff[2];
F32 integral;

/* プロトタイプ宣言 */
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
        /* 倒立制御 */
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

    /* 倒立制御 */
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

    /* 尻尾制御 */
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
    F32 forward = 80.0;
    F32 turn;
    U16 lightness;

    balance_control(
        (F32)10,
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

    //lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    //lt_calc_pid(lightness, &turn);
    //if (turn > 100) {
    //    turn = 100.0;
    //} else if (turn < -100) {
    //    turn = -100.0;
    //}
    //
    ////if (turn > 50.0) {
    ////    forward = 40.0;
    ////} else if (turn < -50.0) {
    ////    forward = 40.0;
    ////}
    //
    ////if (lightness < BLACK_WHITE_THRESHOLD) {
    ////    turn = (F32)80; // 白いときは、右へ
    ////} else {
    ////    turn = (F32)-80; // 黒いときは、左へ
    ////}
    //
    //balance_control(
    //    (F32)forward,
    //    turn,
    //    (F32)ecrobot_get_gyro_sensor(PORT_GYRO),
    //    GYRO_OFFSET,
    //    (F32)nxt_motor_get_count(PORT_MOTOR_L),
    //    (F32)nxt_motor_get_count(PORT_MOTOR_R),
    //    (F32)ecrobot_get_battery_voltage(),
    //    &pwm_l,
    //    &pwm_r);
    //
    //nxt_motor_set_speed(PORT_MOTOR_L, pwm_l, 1);
    //nxt_motor_set_speed(PORT_MOTOR_R, pwm_r, 1);

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
    F32 p, i, d;
    diff[0] = diff[1];
    diff[1] = light - BLACK_WHITE_THRESHOLD;
    integral += (diff[1] + diff[0]) / 2.0 * 0.004;

    p = 2.0 * diff[1];
    i = 0.0 * integral;
    d = 0.0 * (diff[1] - diff[0]) / 0.004;

    //*pturn = ((light - BLACK_WHITE_THRESHOLD) * -1.2);

    *pturn = (p + i + d);
}
