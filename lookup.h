#include "mytypes.h"

#ifndef __LOOKUP_H__
#define __LOOKUP_H__

#define LOOKUP_STAT_INI         (U32)1  /* ‰Šúó‘Ô         */
#define LOOKUP_STAT_TAIL_DOWN   (U32)2  /* K”ö§Œä(~‰º)   */
#define LOOKUP_STAT_COCK        (U32)3  /* ‘–s‘Ì§Œäó‘Ô   */
#define LOOKUP_STAT_TAIL_UP     (U32)4  /* K”ö§Œä(ã¸)   */
#define LOOKUP_STAT_FORWARD     (U32)5  /* ‘Oió‘Ô         */
#define LOOKUP_STAT_STAND       (U32)6  /* “|—§ó‘Ô         */
#define LOOKUP_STAT_RUN         (U32)7

#define LOOKUP_COCK_START_ANGLE (U32)110
#define LOOKUP_COCK_ANGLE       (U32)60

typedef struct _LOOKUP_STAT{
    U32 lookupStat;
} LOOKUP_STAT;

#endif  /* __LOOKUP_H__ */
