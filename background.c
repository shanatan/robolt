#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#include "port_IF.h"

#include "tasksync_IF.h"
#include "sensor_IF.h"

#include "main.h" /* ���O�p */

extern  TASK_MES    gkTaskMes;
extern  SENSOR_VAL  gkSensorVal;

extern  ROBOT_STAT   gkRobotStat; /* ���O�p */

TASK(BackGround)
{
    int moter_r = 0;
    int old_moter_r = 0;
    int moter_l = 0;
    int old_moter_l = 0;
    int diff_cnt = 0;
    int sonar_cnt = 0;

    ecrobot_send_bt("power on", 0, 8);

    for(;;) {
        ///* ���b�N�A�b�v�Q�[�g���m */
        //gkSensorVal.sonar = ecrobot_get_sonar_sensor(PORT_SONAR);
        //
        //if (gkSensorVal.sonar != 255) {
        //    if (gkSensorVal.sonar < 35) {
        //        sonar_cnt++;
        //        if (sonar_cnt > 20) {
        //            gkTaskMes.message = MES_LOOKUP;
        //        } else { /* nothing */ }
        //    } else { /* nothing */ }
        //} else { /* nothing */ }

        /* �Ռ����m */
        old_moter_r = moter_r;
        old_moter_l = moter_l;
        moter_r = nxt_motor_get_count(PORT_MOTOR_R);
        moter_l = nxt_motor_get_count(PORT_MOTOR_L);
        if (moter_r > 10000) {
            if ((moter_r - old_moter_r) < 10 && (moter_l - old_moter_l) < 10 ) {
                diff_cnt++;
                if (diff_cnt > 1) {
                    gkTaskMes.message = MES_SHOCK;
                } else { /* nothing */ }
            } else { /* nothing */ }
        } else { /* nothing */ }

        switch (gkRobotStat.robotStat) {
            case ROBOT_STAT_READY:  /* ��~���                 */
                ecrobot_send_bt("r", 0, 1);
                break;
            case ROBOT_STAT_STAND:  /* �|�����                 */
                ecrobot_send_bt("i", 0, 1);
                break;
            case ROBOT_STAT_TRACE:  /* �g���[�X���             */
                ecrobot_send_bt("t", 0, 1);
                break;
            case ROBOT_STAT_LOOKUP: /* ���b�N�A�b�v�Q�[�g���   */
                ecrobot_send_bt("l", 0, 1);
                break;
            case ROBOT_STAT_SEESAW: /* �V�[�\�[���             */
                ecrobot_send_bt("s", 0, 1);
                break;
            case ROBOT_STAT_GARAGE: /* �K���[�W���             */
                ecrobot_send_bt("g", 0, 1);
                break;
            default:
                break;
        }

        //ecrobot_bt_data_logger((S8)0, (S8)0);

        systick_wait_ms(50);
    }
}
