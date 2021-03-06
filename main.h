#include "mytypes.h"

#ifndef __MAIN_H__
#define __MAIN_H__

#define ROBOT_STAT_READY    (U32)1  /* 停止状態                 */
#define ROBOT_STAT_STAND    (U32)2  /* 倒立状態                 */
#define ROBOT_STAT_TRACE    (U32)3  /* トレース状態             */
#define ROBOT_STAT_LOOKUP   (U32)4  /* ルックアップゲート状態   */
#define ROBOT_STAT_SEESAW   (U32)5  /* シーソー状態             */
#define ROBOT_STAT_GARAGE   (U32)6  /* ガレージ状態             */

typedef struct _ROBOT_STAT{
    U32 robotStat;
} ROBOT_STAT;

#endif  /* __MAIN_H__ */
