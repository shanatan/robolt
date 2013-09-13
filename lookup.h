#include "mytypes.h"

#ifndef __LOOKUP_H__
#define __LOOKUP_H__

#define LOOKUP_STAT_INI         (U32)1  /* èâä˙èÛë‘         */
#define LOOKUP_STAT_TAIL_DOWN   (U32)2  /* êKîˆêßå‰(ç~â∫)   */
#define LOOKUP_STAT_COCK        (U32)3  /* ëñçsëÃêßå‰èÛë‘   */
#define LOOKUP_STAT_TAIL_UP     (U32)4  /* êKîˆêßå‰(è„è∏)   */
#define LOOKUP_STAT_SEARCH_EDGE (U32)5  /* ì|óßèÛë‘         */
#define LOOKUP_STAT_FORWARD     (U32)6  /* ëOêièÛë‘         */
#define LOOKUP_STAT_TURN_L      (U32)7
#define LOOKUP_STAT_BACK        (U32)8  /* ì|óßèÛë‘         */
#define LOOKUP_STAT_TURN_R      (U32)9
#define LOOKUP_STAT_FORWARD_2ND (U32)10

#define LOOKUP_COCK_START_ANGLE (U32)110
#define LOOKUP_COCK_ANGLE       (U32)67

typedef struct _LOOKUP_STAT{
    U32 lookupStat;
} LOOKUP_STAT;

#endif  /* __LOOKUP_H__ */
