#ifndef __RETURN_H__
#define __RETURN_H__

#define RET_OK      (U32)0x00000000 /* 正常終了             */
#define RET_NG      (U32)0x00000001 /* 異常終了             */
#define RET_REMAIN  (U32)0x00000011 /* 正常終了(処理未完)   */
#define RET_FINISH  (U32)0x00000021 /* 正常終了(処理完了)   */
#define RET_FATAL   (U32)0xFFFFFFFF /* 致命的エラー発生     */

#endif  /* __RETURN_H__ */
