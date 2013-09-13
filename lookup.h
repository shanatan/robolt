#include "mytypes.h"

#ifndef __LOOKUP_H__
#define __LOOKUP_H__

#define LOOKUP_STAT_INI         (U32)1  /* 初期状態         */
#define LOOKUP_STAT_TAIL_DOWN   (U32)2  /* 尻尾制御(降下)   */
#define LOOKUP_STAT_COCK        (U32)3  /* 走行体制御状態   */
#define LOOKUP_STAT_TAIL_UP     (U32)4  /* 尻尾制御(上昇)   */
#define LOOKUP_STAT_SEARCH_EDGE (U32)5  /* 倒立状態         */
#define LOOKUP_STAT_FORWARD     (U32)6  /* 前進状態         */
#define LOOKUP_STAT_TURN_L      (U32)7
#define LOOKUP_STAT_BACK        (U32)8  /* 倒立状態         */
#define LOOKUP_STAT_TURN_R      (U32)9
#define LOOKUP_STAT_FORWARD_2ND (U32)10

#define LOOKUP_COCK_START_ANGLE (U32)110
#define LOOKUP_COCK_ANGLE       (U32)67

typedef struct _LOOKUP_STAT{
    U32 lookupStat;
} LOOKUP_STAT;

#endif  /* __LOOKUP_H__ */
