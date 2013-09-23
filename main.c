#include "kernel.h"
#include "kernel_id.h"

#include "mytypes.h"

#include "return.h"

#include "main.h"
#include "main_IF.h"
#include "tasksync_IF.h"

#include "ecrobot_interface.h"

#include "linetrace_IF.h"
#include "lookup_IF.h"
#include "seesaw_IF.h"
#include "garage_IF.h"

extern ROBOT_STAT   gkRobotStat;
extern TASK_MES     gkTaskMes;

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
TASK(LineTrace)
{
    U32 ret = RET_REMAIN;

    /* ��ԑJ�� */
    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* ��~���                 */
        case ROBOT_STAT_STAND:  /* �|�����                 */
            switch (gkTaskMes.message) {
                case MES_LOOKUP:
                case MES_SHOCK:
                    gkTaskMes.message = MES_INI;
                    break;
                default:
                    break;
            }
            break;
        case ROBOT_STAT_TRACE:  /* �g���[�X���             */
            switch (gkTaskMes.message) {
                case MES_LOOKUP:    gkRobotStat.robotStat = ROBOT_STAT_LOOKUP;  break;
                case MES_SHOCK:     gkRobotStat.robotStat = ROBOT_STAT_SEESAW;  break;
                default:                                                        break;
            }
            break;
        case ROBOT_STAT_LOOKUP: /* ���b�N�A�b�v�Q�[�g���   */ break;
        case ROBOT_STAT_SEESAW: /* �V�[�\�[���             */ break;
        case ROBOT_STAT_GARAGE: /* �K���[�W���             */ break;
        default:
            break;
    }

    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* ��~���                 */
            lt_pol_triger();
            break;
        case ROBOT_STAT_STAND:  /* �|�����                 */
            lt_statnd();
            break;
        case ROBOT_STAT_TRACE:  /* �g���[�X���             */
            lt_linetrace();
            break;
        case ROBOT_STAT_LOOKUP: /* ���b�N�A�b�v�Q�[�g���   */
            ret = lookup_entry();
            break;
        case ROBOT_STAT_SEESAW: /* �V�[�\�[���             */
            ret = seesaw_entry();
            break;
        case ROBOT_STAT_GARAGE: /* �K���[�W���             */
            garage_entry();
            break;
        default:
            break;
    }

    if (ret == RET_FINISH) {
        switch (gkRobotStat.robotStat) {
            case ROBOT_STAT_READY:                                              break;
            case ROBOT_STAT_STAND:                                              break;
            case ROBOT_STAT_TRACE:                                              break;
            case ROBOT_STAT_LOOKUP: gkRobotStat.robotStat = ROBOT_STAT_GARAGE;  break;
            case ROBOT_STAT_SEESAW: gkRobotStat.robotStat = ROBOT_STAT_GARAGE;  break;
            case ROBOT_STAT_GARAGE:                                             break;
            default:                                                            break;
        }
    } else { /* nothing */ }

    TerminateTask();
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
void    lt_ini(
            void
        )
{
    gkRobotStat.robotStat = ROBOT_STAT_READY;
    gkTaskMes.message = MES_INI;
}
