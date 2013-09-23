#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include "balancer.h"

#include "return.h"

#include "port_IF.h"
#include "sensor_IF.h"

#include "lookup.h"
#include "lookup_IF.h"

extern  LOOKUP_STAT gkLookupStat;
extern  SENSOR_VAL  gkSensorVal;

/* ÉvÉçÉgÉ^ÉCÉvêÈåæ */
static  U32     lookup_start(void);
static  U32     lookup_tail_down(void);
static  U32     lookup_cock(void);
static  U32     lookup_search_edge(void);
static  U32     lookup_forward(void);
static  U32     lookup_turn_l(void);
static  U32     lookup_back(void);
static  U32     lookup_turn_r(void);
static  U32     lookup_forward_2nd(void);
static  U32     lookup_up(void);

static  void    lookup_stand(F32 foward, F32 turn, F32 gyro_offset);

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
void            lookup_ini(
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
U32             lookup_entry(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U32 main_ret    = RET_REMAIN;

    /* êUÇÈïëÇ¢îªíË */
    switch (gkLookupStat.lookupStat) {
        case LOOKUP_STAT_INI:           ret = lookup_start();       break;
        case LOOKUP_STAT_TAIL_DOWN:     ret = lookup_tail_down();   break;
        case LOOKUP_STAT_COCK:          ret = lookup_cock();        break;
        case LOOKUP_STAT_SEARCH_EDGE:   ret = lookup_search_edge(); break;
        case LOOKUP_STAT_FORWARD:       ret = lookup_forward();     break;
        case LOOKUP_STAT_TURN_L:        ret = lookup_turn_l();      break;
        case LOOKUP_STAT_BACK:          ret = lookup_back();        break;
        case LOOKUP_STAT_TURN_R:        ret = lookup_turn_r();      break;
        case LOOKUP_STAT_FORWARD_2ND:   ret = lookup_forward_2nd(); break;
        case LOOKUP_STAT_UP:            ret = lookup_up();          break;
        default:                                                    break;
    }

    /* èÛë‘ëJà⁄ */
    switch (ret) {
        case RET_REMAIN:    /* nothing */   break;
        case RET_FINISH:
            switch (gkLookupStat.lookupStat) {
                case LOOKUP_STAT_INI:           gkLookupStat.lookupStat = LOOKUP_STAT_TAIL_DOWN;    break;
                case LOOKUP_STAT_TAIL_DOWN:     gkLookupStat.lookupStat = LOOKUP_STAT_COCK;         break;
                case LOOKUP_STAT_COCK:          gkLookupStat.lookupStat = LOOKUP_STAT_SEARCH_EDGE;  break;
                case LOOKUP_STAT_SEARCH_EDGE:   gkLookupStat.lookupStat = LOOKUP_STAT_FORWARD;      break;
                case LOOKUP_STAT_FORWARD:       gkLookupStat.lookupStat = LOOKUP_STAT_TURN_L;       break;
                case LOOKUP_STAT_TURN_L:        gkLookupStat.lookupStat = LOOKUP_STAT_BACK;         break;
                case LOOKUP_STAT_BACK:          gkLookupStat.lookupStat = LOOKUP_STAT_TURN_R;       break;
                case LOOKUP_STAT_TURN_R:        gkLookupStat.lookupStat = LOOKUP_STAT_FORWARD_2ND;  break;
                case LOOKUP_STAT_FORWARD_2ND:   gkLookupStat.lookupStat = LOOKUP_STAT_UP;           break;
                case LOOKUP_STAT_UP:            main_ret                = RET_FINISH;               break;
                default:                                                                            break;
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
**********************************************************/
static  U32     lookup_start(
                    void
                )
{
    U32         ret         = RET_REMAIN;
    U16         lightness;
    F32         turn;
    F32         forward;
    static int  old_sonar   = 35;

    old_sonar = (gkSensorVal.sonar + old_sonar) / 2;
    forward = (1.0) * (old_sonar - 20);

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    turn = (1.5) * (530 - lightness);

    lookup_stand(forward, turn, GYRO_OFFSET);

    if (forward <= 0) {
        nxt_motor_set_count(PORT_MOTOR_TAIL, 0);
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)10, 1);
        ret = RET_FINISH;
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
static  U32     lookup_tail_down(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U16 lightness;
    F32 turn;
    F32 forward;
    int tail;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    turn = (0.5) * (530 - lightness);
    forward = (0.7) * (gkSensorVal.sonar - 15);
    lookup_stand((F32)forward, (F32)turn, GYRO_OFFSET);

    /* êKîˆêßå‰ */
    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
    if (tail > LOOKUP_COCK_START_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)0, 1);
        lookup_stand((F32)0, (F32)0, GYRO_OFFSET - 40);
        ret = RET_FINISH;
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
static  U32     lookup_cock(
                    void
                )
{
    U32 ret     = RET_REMAIN;
    int tail;

    tail = nxt_motor_get_count(PORT_MOTOR_TAIL);

    if (tail > LOOKUP_COCK_ANGLE) {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)-5, 1);
    } else {
        nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)40, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        ret = RET_FINISH;
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
static  U32     lookup_search_edge(
                    void
                )
{
    U32         ret         = RET_REMAIN;
    U16         lightness;
    static  U32 cnt         = 0;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);
    if (lightness > 630) {
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        nxt_motor_set_count(PORT_MOTOR_R, 0);
        nxt_motor_set_count(PORT_MOTOR_L, 0);
        ret = RET_FINISH;
    } else {
        cnt++;
        if (cnt < 250) {
            nxt_motor_set_speed(PORT_MOTOR_R, 30, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, -30, 1);
        } else {
            nxt_motor_set_speed(PORT_MOTOR_R, -30, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, 30, 1);
        }
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
 *********************************************************/
static  U32     lookup_forward(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U16 lightness;
    U32 forward;
    int motor_r;
    int motor_l;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);

    nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)30, 1);

    if (lightness > 600) {
        forward = 40 + (1.0) * (lightness - 600);
        nxt_motor_set_speed(PORT_MOTOR_R, forward, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 20, 1);
    } else {
        forward = 40 + (1.0) * (600 - lightness);
        nxt_motor_set_speed(PORT_MOTOR_R, 20, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, forward, 1);
    }

    motor_r = nxt_motor_get_count(PORT_MOTOR_R);
    motor_l = nxt_motor_get_count(PORT_MOTOR_L);
    if (motor_r > 630 && motor_l > 630) {
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        ret = RET_FINISH;
    } else {}

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
 *********************************************************/
static  U32     lookup_turn_l(
                    void
                )
{
    U32         ret         = RET_REMAIN;
    U16         lightness;
    static  U32 cnt         = 0;

    cnt++;
    if (cnt < 250) {
        nxt_motor_set_speed(PORT_MOTOR_R, 30, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, -30, 1);
    } else {
        lightness = ecrobot_get_light_sensor(PORT_LIGHT);
        if (lightness > 600) {
            nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
            nxt_motor_set_count(PORT_MOTOR_R, 0);
            nxt_motor_set_count(PORT_MOTOR_L, 0);
            ret = RET_FINISH;
        } else {
            nxt_motor_set_speed(PORT_MOTOR_R, 30, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, -30, 1);
        }
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
 *********************************************************/
static  U32     lookup_back(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U16 lightness;
    U32 forward;
    int motor_r;
    int motor_l;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);

    nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)30, 1);

    if (lightness < 600) {
        forward = 40 + (1.0) * (600 - lightness);
        nxt_motor_set_speed(PORT_MOTOR_R, forward, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 20, 1);
    } else {
        forward = 40 + (1.0) * (lightness - 600);
        nxt_motor_set_speed(PORT_MOTOR_R, 20, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, forward, 1);
    }

    motor_r = nxt_motor_get_count(PORT_MOTOR_R);
    motor_l = nxt_motor_get_count(PORT_MOTOR_L);
    if (motor_r > 630 && motor_l > 630) {
        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        ret = RET_FINISH;
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
 *********************************************************/
static  U32     lookup_turn_r(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U16 lightness;
    static  U32 cnt = 0;

    cnt++;
    if (cnt < 250) {
        nxt_motor_set_speed(PORT_MOTOR_R, -30, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 30, 1);
    } else {
        lightness = ecrobot_get_light_sensor(PORT_LIGHT);
        if (lightness > 600) {
            nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
            nxt_motor_set_count(PORT_MOTOR_R, 0);
            nxt_motor_set_count(PORT_MOTOR_L, 0);
            ret = RET_FINISH;
        } else {
            nxt_motor_set_speed(PORT_MOTOR_R, -30, 1);
            nxt_motor_set_speed(PORT_MOTOR_L, 30, 1);
        }
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
 *********************************************************/
static  U32     lookup_forward_2nd(
                    void
                )
{
    U32 ret         = RET_REMAIN;
    U16 lightness;
    U32 forward;
    int motor_r;

    lightness = ecrobot_get_light_sensor(PORT_LIGHT);

    nxt_motor_set_speed(PORT_MOTOR_TAIL, (S8)30, 1);

    if (lightness > 600) {
        forward = 40 + (1.0) * (lightness - 600);
        nxt_motor_set_speed(PORT_MOTOR_R, forward, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 20, 1);
    } else {
        forward = 40 + (1.0) * (600 - lightness);
        nxt_motor_set_speed(PORT_MOTOR_R, 20, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, forward, 1);
    }

    motor_r = nxt_motor_get_count(PORT_MOTOR_R);
    if (motor_r > 900) {
        nxt_motor_set_count(PORT_MOTOR_TAIL, 0);

        nxt_motor_set_speed(PORT_MOTOR_R, 0, 1);
        nxt_motor_set_speed(PORT_MOTOR_L, 0, 1);
        nxt_motor_set_count(PORT_MOTOR_R, 0);
        nxt_motor_set_count(PORT_MOTOR_L, 0);
        balance_init();
        ret = RET_FINISH;
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
 *********************************************************/
static  U32     lookup_up(
                    void
                )
{
    U32         ret     = RET_REMAIN;
    int         tail;
    static  int flg     = 0;
    int         power;
    static  U32 cnt     = 0;

    if (flg == 0) {
        tail = nxt_motor_get_count(PORT_MOTOR_TAIL);
        if (tail < 52) {
            power = (0.25 * (52 - tail)) + 60;
          /* êKîˆÇâ∫Ç∞Çƒì|óßÇ≥ÇπÇÈ */
            nxt_motor_set_speed(PORT_MOTOR_TAIL, power, 1);
        } else {
            flg = 1;
        }
    } else {
        cnt++;
        nxt_motor_set_speed(PORT_MOTOR_TAIL, -40, 1);
        lookup_stand((F32)0, (F32)0, GYRO_OFFSET);
        if (cnt > 1500) {
            ret = RET_FINISH;
        } else {}
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
static  void    lookup_stand(
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
