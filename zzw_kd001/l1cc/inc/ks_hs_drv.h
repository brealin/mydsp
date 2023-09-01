/***********************************************************************************************************************
* Copyright (C) 2019 Shanghai Kindroid Network Tech Ltd. All rights reserved
*
* License:
*
* Description:
*
* Author:
*
* Revision History:
* 1.0 2019-06-17 created by can.yin
***********************************************************************************************************************/
#ifndef _KS_HS_DRV_H_
#define _KS_HS_DRV_H_
#include "ks_oal.h"
#include "ks_phych_type.h"
#include "ks_phych_func.h"



extern UINT16 dsp2arm_buffer_index;
#if 0
#define PHYCH_DATA_BUFNUM              512
#define QPSK_PACKAGE_MAX_SIZE          1024
#define BPSK_PACKAGE_MAX_SIZE          4096
#define QAM16_PACKAGE_MAX_SIZE         6144

    
typedef struct phych_frame_tag 
{   
    UINT8  reserved[20];
    UINT16 index;
	UINT16 len;
} phych_frame_t;

typedef struct phych_head_tag
{
    UINT32 u32_read_offset;         /* 璇诲彇鏂圭殑璇诲彇浣嶇疆锛屾暟鍊间负閽堝鍏变韩鍐呭瓨璧峰浣嶇疆鐨勫亸绉婚噺锛屽崟浣岯yte */
    UINT32 u32_read_irq_num;        /* 璇诲彇鏂瑰凡缁忔敹鍒扮殑涓柇涓暟 */
    UINT32 u32_read_data_count;     /* 璇诲彇鏂瑰凡缁忚鍙栫殑鏁版嵁鎬婚噺锛屽崟浣岯yte */
    UINT32 u32_read_reserved;       /* 璇诲彇鏂逛繚鐣欎綅 */

    UINT32 u32_write_offset;        /* 鍙戦�佹柟鐨勫啓鍏ヤ綅缃紝鏁板�间负閽堝鍏变韩鍐呭瓨璧峰浣嶇疆鐨勫亸绉婚噺锛屽崟浣岯yte */
    UINT32 u32_write_irq_num;       /* 鍙戦�佹柟宸茬粡瑙﹀彂鐨勪腑鏂釜鏁� */
    UINT32 u32_write_data_count;    /* 鍙戦�佹柟宸茬粡鍙戦�佺殑鏁版嵁鎬婚噺锛屽崟浣岯yte */
    UINT32 u32_write_reserved;      /* 鍙戦�佹柟淇濈暀浣� */
} phych_head_t;

typedef struct pkg_frame_tag 
{   
    UINT16 index;
    UINT16 len;
} pkg_frame_t;
#endif

typedef struct pkg_head_tag 
{   
    UINT32      pkg_num;
    UINT32      pkg_max_size;
    UINT32      pkg_valid_size;
} pkg_head_t;

#if defined(CORE_X1643)
extern pkg_head_t g_pkg_head;
#endif

#if defined(CORE_X1643)
//  UINT16 x1643_get_unread_buf_info(UINT32 pkg_max_size);
//  void x1643_phych_read_data(pkg_head_t *pkg_ptr, UINT8_PTR dest_addr);
// void x1643_phych_read_mbuf(pkg_head_t *pkg_ptr, UINT8_PTR dest_addr);
#else
void xc4500_phych_proc_data(UINT8_PTR src, UINT32 u32_frame_type, SINT32 s32_frame_no, SINT32 s32_rtc_value, UINT16 pkg_type, UINT16 pkg_len);
#endif

#endif
/***************************************************** END OF FILE ****************************************************/

