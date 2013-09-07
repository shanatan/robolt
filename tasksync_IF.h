#include "mytypes.h"

#ifndef __TASKSYNC_IF_H__
#define __TASKSYNC_IF_H__

#define MES_INI     (U32)1  /* 初期状態                 */
#define MES_LOOKUP  (U32)2  /* ルックアップゲート発見   */

typedef struct _TASK_MES{
    U32 message;
} TASK_MES;

#endif  /* __TASKSYNC_IF_H__ */
