#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"

#include "port_IF.h"
#include "sensor_IF.h"
#include "return.h"

#include "seesaw.h"
#include "seesaw_IF.h"

extern  SEESAW_STAT gkLSeesawStat;

/* ÉvÉçÉgÉ^ÉCÉvêÈåæ */
static  U32     seesaw_forward(void);
static  U32     seesaw_stop(void);

static  void    seesaw_stand(F32 foward, F32 turn, F32 gyro_offset);

/*!*********************************************************
 *  @brief
 *
 *  @param
 *
 *  @retval
 *
 *  @return
 *
 *********************************************************/
void            seesaw_ini(
                    void
                )
{
    gkLSeesawStat.SeesawStat = SEESAW_STAT_INI;

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
 *********************************************************/
U32             seesaw_entry(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U32 main_ret    = RET_REMAIN;

    /* êUÇÈïëÇ¢îªíË */
    switch (gkLSeesawStat.SeesawStat) {
        case SEESAW_STAT_INI:   ret = seesaw_forward(); break;
        case SEESAW_STAT_BACK:  ret = seesaw_stop();    break;
        default:                                        break;
    }

    /* èÛë‘ëJà⁄ */
    switch (ret) {
        case RET_REMAIN:    /* nothing */   break;
        case RET_FINISH:
            switch (gkLSeesawStat.SeesawStat) {
                case SEESAW_STAT_INI:   main_ret = RET_FINISH;   break;
                case SEESAW_STAT_BACK:  main_ret = RET_FINISH;   break;
                default:                                    break;
            }
            break;
        default:            /* nothing */   break;
    }

    return main_ret;
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
 *********************************************************/
static  U32     seesaw_forward(
                    void
                )
{
    U32         ret             = RET_REMAIN;
    U16         lightness;
    F32         turn;
    static  int motor_r         = 0;
    static  int motor_l         = 0;
    int         cur_motor_r;
    int         cur_motor_l;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    turn = (0.2) * (530 - lightness);

    seesaw_stand((F32)15, (F32)turn, GYRO_OFFSET + 2);
    //seesaw_stand((F32)1, (F32)0, GYRO_OFFSET);

    if (motor_r == 0) {
        motor_r = nxt_motor_get_count(PORT_MOTOR_R);
        motor_l = nxt_motor_get_count(PORT_MOTOR_L);
        nxt_motor_set_count(PORT_MOTOR_TAIL, 0);
    } else {
        ///* êKîˆêßå‰ */
        //tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
        //if (tail < SEESAW_TAIL_ANGLE_BLAKE) {
        //    nxt_motor_set_speed(PORT_MOTOR_TAIL, 20, 1);
        //} else {
        //    nxt_motor_set_speed(PORT_MOTOR_TAIL, 0, 1);
        //}

        cur_motor_r = nxt_motor_get_count(PORT_MOTOR_R);
        cur_motor_l = nxt_motor_get_count(PORT_MOTOR_L);
        if (cur_motor_r - motor_r > 900 && cur_motor_l - motor_l > 900) {
            ret = RET_FINISH;
        } else { /* nothing */ }
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
static  U32     seesaw_stop(
                    void
                )
{
    U32 ret         = RET_REMAIN;

    seesaw_stand((F32)0, (F32)0, GYRO_OFFSET);

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
static  void    seesaw_stand(
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
