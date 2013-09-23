#include "mytypes.h"

#ifndef __GARAGE_H__
#define __GARAGE_H__


#define GAGAGE_RET_FORE         (U32)0  /* �|���U�qAPI�߂�(�O�i)    */
#define GAGAGE_RET_REAR         (U32)1  /* �|���U�qAPI�߂�(���)    */

#define GAGAGE_STAT_INI         (U32)1  /* �������     */
#define GAGAGE_STAT_TAIL_DOWN   (U32)2  /* �K���~����� */
#define GAGAGE_STAT_STOP        (U32)3  /* ��~���     */

typedef struct _GARAGE_MNG_INFO{
    U32 garageStat;
    U32 motor_r;
    U32 motor_l;
} GARAGE_MNG_INFO;

#define GARAGE_COCK_START_ANGLE (U32)105

#endif  /* __GARAGE_H__ */
