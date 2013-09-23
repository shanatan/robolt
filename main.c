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

    /* 状態遷移 */
    switch (gkRobotStat.robotStat) {
        case ROBOT_STAT_READY:  /* 停止状態                 */
        case ROBOT_STAT_STAND:  /* 倒立状態                 */
            switch (gkTaskMes.message) {
                case MES_LOOKUP:
                case MES_SHOCK:
                    gkTaskMes.message = MES_INI;
                    break;
                default:
                    break;
            }
            break;
        case ROBOT_STAT_TRACE:  /* トレース状態             */
            switch (gkTaskMes.message) {
                case MES_LOOKUP:    gkRobotStat.robotStat = ROBOT_STAT_LOOKUP;  break;
                case MES_SHOCK:     gkRobotStat.robotStat = ROBOT_STAT_SEESAW;  break;
                default:                                                        break;
            }
            break;
        case ROBOT_STAT_LOOKUP: /* ルックアップゲート状態   */ break;
        case ROBOT_STAT_SEESAW: /* シーソー状態             */ break;
        case ROBOT_STAT_GARAGE: /* ガレージ状態             */ break;
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
            ret = lookup_entry();
            break;
        case ROBOT_STAT_SEESAW: /* シーソー状態             */
            ret = seesaw_entry();
            break;
        case ROBOT_STAT_GARAGE: /* ガレージ状態             */
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
