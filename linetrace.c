#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "sensor_IF.h"

#include "main.h"
#include "linetrace.h"
#include "linetrace_IF.h"

extern  ROBOT_STAT  gkRobotStat;

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
    U8  touch;
    U32 bt = 0;
    char buf[128];
    U32 size;
    S8  pwm_l;
    S8  pwm_r;

    /* タッチセンサ */
    touch = ecrobot_get_touch_sensor(PORT_TOUCH);

    /* Bluetooth */
    size = ecrobot_read_bt(buf, 0, 128);
    if (size > 0) {
        if (buf[0] == 's') {
            ecrobot_send_bt("start", 0,  5);
            bt = 1;
        } else {
            ecrobot_send_bt("unknown", 0, 7);
        }
    } else { /* nothing */ }

    if (touch == 1 || bt == 1) {
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
        //gkRobotStat.robotStat = ROBOT_STAT_GARAGE;
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

    /* 絶対値をとる */
    if (turn > 0) {
        u_turn = turn;
    } else {
        u_turn = (-1) * turn;
    }

    /* 旋回量から前進量を調整する */
    if (u_turn < 30) {
        forward = 85;
    } else if (u_turn < 50) {
        forward = 85;
    } else {
        forward = 85;
    }

    //if (u_turn < 30) {
    //    forward = 100;
    //} else if (u_turn < 50) {
    //    forward = 90;
    //} else {
    //    forward = 85;
    //}

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
    static F32  integral = 0;
    static U32  cnt = 0;

    F32 p;
    F32 i;
    F32 d;

    /* 積算をクリアする */
    cnt++;
    if (cnt > (INTEGRAL_CLEAR_TIME / (U32)4)) {
        integral = (F32)0;
        cnt = 0;
    } else { /* nothing */ }

    diff[0] = diff[1];
    diff[1] = light - BLACK_WHITE_THRESHOLD;
    integral += (diff[1] + diff[0]) / 2.0 * 0.004;

    p = 1.50 * diff[1];
    i = 0.50 * integral;
    d = 0.40 * (diff[1] - diff[0]) / 0.004;

    //*pturn = ((light - BLACK_WHITE_THRESHOLD) * -1.2);

    *pturn = (-1) * (p + i + d);
}
