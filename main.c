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
    /* 状態遷移 */
    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* 停止状態                 */
        case ROBOT_STAT_STAND:  /* 倒立状態                 */
            if (gkTaskMes.message == MES_LOOKUP) {
                gkTaskMes.message = MES_INI;
            } else { /* nothing */ }
            break;
        case ROBOT_STAT_TRACE:  /* トレース状態             */
            if (gkTaskMes.message == MES_LOOKUP) {
                gkRobotStat.robotStat = ROBOT_STAT_LOOKUP;
            } else { /* nothing */ }
            break;
        case ROBOT_STAT_LOOKUP: /* ルックアップゲート状態   */ break;
        default:
            break;
    }

    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* 停止状態                 */
            lt_pol_triger();
            break;
        case ROBOT_STAT_STAND:  /* 倒立状態                 */
            lt_statnd();
            break;
        case ROBOT_STAT_TRACE:  /* トレース状態             */
            lt_linetrace();
            break;
        case ROBOT_STAT_LOOKUP: /* ルックアップゲート状態   */
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
