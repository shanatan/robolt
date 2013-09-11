#include "kernel.h"
#include "kernel_id.h"

#include "mytypes.h"

#include "main.h"
#include "main_IF.h"
#include "tasksync_IF.h"

#include "linetrace_IF.h"
#include "lookup_IF.h"

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
    /* ��ԑJ�� */
    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* ��~���                 */
        case ROBOT_STAT_STAND:  /* �|�����                 */
            if (gkTaskMes.message == MES_LOOKUP) {
                gkTaskMes.message = MES_INI;
            } else { /* nothing */ }
            break;
        case ROBOT_STAT_TRACE:  /* �g���[�X���             */
            if (gkTaskMes.message == MES_LOOKUP) {
                gkRobotStat.robotStat = ROBOT_STAT_LOOKUP;
            } else { /* nothing */ }
            break;
        case ROBOT_STAT_LOOKUP: /* ���b�N�A�b�v�Q�[�g���   */ break;
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
            lookup_entry();
            break;
        default:
            break;
    }

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
