#include "mytypes.h"

#ifndef __LOOKUP_H__
#define __LOOKUP_H__

#define LOOKUP_STAT_INI         (U32)1  /* 初期状態         */
#define LOOKUP_STAT_TAIL_DOWN   (U32)2  /* 尻尾制御(降下)   */
#define LOOKUP_STAT_COCK        (U32)3  /* 走行体制御状態   */
#define LOOKUP_STAT_TAIL_UP     (U32)4  /* 尻尾制御(上昇)   */
#define LOOKUP_STAT_FORWARD     (U32)5  /* 前進状態         */
#define LOOKUP_STAT_STAND       (U32)6  /* 倒立状態         */
#define LOOKUP_STAT_RUN         (U32)7

#define LOOKUP_COCK_START_ANGLE (U32)110
#define LOOKUP_COCK_ANGLE       (U32)80

typedef struct _LOOKUP_STAT{
    U32 lookupStat;
} LOOKUP_STAT;

#endif  /* __LOOKUP_H__ */
