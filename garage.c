#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"

#include "return.h"
#include "port_IF.h"
#include "sensor_IF.h"

#include "garage.h"
#include "garage_IF.h"

extern  GARAGE_MNG_INFO gkGarageInfo;

static  U32     garage_trace(void);
static  U32     garage_tail_down(void);
static  U32     garage_stop(void);
static  U32     garage_stand(F32 foward, F32 turn, F32 gyro_offset);
static  void    garage_calc_pid( U16 light, F32 *pturn);

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
void            garage_ini(
                    void
                )
{
    gkGarageInfo.garageStat = GAGAGE_STAT_INI;

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
void            garage_entry(
                    void
                )
{
    U32 ret = RET_REMAIN;

    /* êUÇÈïëÇ¢îªíË */
    switch (gkGarageInfo.garageStat) {
        case GAGAGE_STAT_INI:       ret = garage_trace();       break;
        case GAGAGE_STAT_TAIL_DOWN: ret = garage_tail_down();   break;
        case GAGAGE_STAT_STOP:      ret = garage_stop();        break;
        default:                                                break;
    }

    /* èÛë‘ëJà⁄ */
    switch (ret) {
        case RET_REMAIN:    /* nothing */   break;
        case RET_FINISH:
            switch (gkGarageInfo.garageStat) {
                case GAGAGE_STAT_INI:       gkGarageInfo.garageStat = GAGAGE_STAT_TAIL_DOWN;    break;
                case GAGAGE_STAT_TAIL_DOWN: gkGarageInfo.garageStat = GAGAGE_STAT_STOP;         break;
                case GAGAGE_STAT_STOP:                                                          break;
                default:                                                                        break;
            }
            break;
        default:            /* nothing */   break;
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
static  U32     garage_trace(
                    void
                )
{

    U32 ret         = RET_REMAIN;
    F32 forward;
    F32 turn;
    F32 u_turn;
    U16 lightness;
    static  U32 cnt = 0;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);

    garage_calc_pid(lightness, &turn);

    /* ê˘âÒó ÇÉtÉBÉãÉ^ÉäÉìÉO */
    if (turn > 100) {
        turn = 100;
    } else if (turn < -100) {
        turn = -100;
    }

    /* ê˘âÒó ÇÃê‚ëŒílÇÇ∆ÇÈ */
    if (turn > 0) {
        u_turn = turn;
    } else {
        u_turn = (-1) * turn;
    }

    cnt++;
    if (cnt > 250) {
        /* ê˘âÒó Ç©ÇÁëOêió Çí≤êÆÇ∑ÇÈ */
        if (u_turn < 30) {
            forward = 100;
        } else if (u_turn < 50) {
            forward = 90;
        } else {
            forward = 85;
        }
        /* ì|óßêUéqêßå‰ */
        garage_stand(forward, turn, GYRO_OFFSET);
        /* äDêFåüím */
        if (lightness > 610) {
            nxt_motor_set_count(PORT_MOTOR_TAIL, 0);
            nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)10, 1);
            gkGarageInfo.motor_r = nxt_motor_get_count(PORT_MOTOR_R);
            gkGarageInfo.motor_l = nxt_motor_get_count(PORT_MOTOR_L);
            ret = RET_FINISH;
        } else { /* nothing */ }
    } else {
        forward = 50;
        /* ì|óßêUéqêßå‰ */
        garage_stand(forward, turn, GYRO_OFFSET);
    }

    return ret;
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
static  U32     garage_tail_down(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U32 garage_ret;
    int tail;
    int motor_r;
    int motor_l;
    F32 forward;
    static U32 cnt  = 0;

    motor_r = nxt_motor_get_count(PORT_MOTOR_R);
    motor_l = nxt_motor_get_count(PORT_MOTOR_L);
    if (motor_r - gkGarageInfo.motor_r < 100 || motor_l - gkGarageInfo.motor_l < 100) {
        forward = 1;
    } else {
        forward = -1;
    }

    garage_ret = garage_stand(forward, 0, GYRO_OFFSET);

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail > GARAGE_COCK_START_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 1);

        if ((motor_r - gkGarageInfo.motor_r) > 100 || (motor_l - gkGarageInfo.motor_l) > 100) {
            cnt++;
            if (cnt > 50) {
                if (garage_ret == GAGAGE_RET_REAR) {
                    //garage_stand((F32)0, (F32)0, (GYRO_OFFSET - 40));
                    nxt_motor_set_speed(PORT_MOTOR_R, (S8)0, 1);
                    nxt_motor_set_speed(PORT_MOTOR_L, (S8)0, 1);
                    ecrobot_set_light_sensor_inactive(PORT_LIGHT);
                    ret = RET_FINISH;
                } else { /* nothing */ }
            } else { /* nothing */ }
        }
    } else { /* nothing */ }

    return ret;
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
static  U32     garage_stop(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    int tail;

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail > GARAGE_COCK_START_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-5, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)5, 1);
    }

    return ret;
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
static  U32     garage_stand(
                    F32 foward,
                    F32 turn,
                    F32 gyro_offset
                )
{
    S8  pwm_l;
    S8  pwm_r;
    U32 ret     = GAGAGE_RET_FORE;

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

    if (pwm_l < 0 && pwm_r < 0) {
        ret = GAGAGE_RET_REAR;
    } else { /* nothing */ }

    return ret;
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
static  void    garage_calc_pid(
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

    /* êœéZÇÉNÉäÉAÇ∑ÇÈ */
    cnt++;
    if (cnt > (1000 / (U32)4)) {
        integral = (F32)0;
        cnt = 0;
    } else { /* nothing */ }

    diff[0] = diff[1];
    diff[1] = light - 555;
    integral += (diff[1] + diff[0]) / 2.0 * 0.004;

    p = 1.50 * diff[1];
    i = 0.50 * integral;
    d = 0.40 * (diff[1] - diff[0]) / 0.004;

    //*pturn = ((light - BLACK_WHITE_THRESHOLD) * -1.2);

    *pturn = (-1) * (p + i + d);
}
