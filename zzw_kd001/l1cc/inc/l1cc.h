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
#ifndef _L1CC_H_
#define _L1CC_H_
#include "ks_oal.h"
#include "zzw_common.h"
/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/
#define L1CC_HLS_MSG_HEAD_SIZE ((UINT16)sizeof(oal_hls_msg_t) - (UINT16)sizeof(VOID_PTR))

#define GEMINI_CP0
#ifdef GEMINI_CP0
#define DDR_RAMBASE   0x80000000 
#else
#define DDR_RAMBASE   0x88000000
#endif

#if defined(CP1_X1643)
#define CORE_X1643_DTCM         ((uint32)0x31000000)
#else
#define CORE_X1643_DTCM         ((uint32)0x11000000)
#endif

    
#define BUFFER_CPARM2DSP_UL_INDEX_0    (DDR_RAMBASE + 0x0636C000) 
#define BUFFER_CPARM2DSP_UL_INDEX_1    (DDR_RAMBASE + 0x0636E000)

#define TX_PWR_INVALID    			   (100)
 
/***********************************************************************************************************************
* GLOBAL FUNCTION DECLARATION
***********************************************************************************************************************/
VOID l1cc_hl_proc_main(VOID);
VOID l1cc_dispatch_main(VOID);

#ifdef CORE_X1643
extern UINT32 g_zzw_ssap;
extern UINT8 g_zzw_sf_b[1134];
extern UINT8 g_u8_node_id_target;
VOID l1cc_ul_proc_main(VOID);
VOID l1cc_ul_mod_main(VOID);
#else
VOID l1cc_dl_proc_main(VOID);
#endif
extern node_info_tbl_t s_st_node_info_tbl;
extern UINT8 node_slot_alloc_query(IN UINT8 u8_ssap, IN UINT32 u32_fn);
extern VOID node_tbl_add();//(IN node_info_t *stp_node_info_in);
extern VOID_PTR node_tbl_find(IN UINT8 u8_node_id);
extern VOID node_tbl_clear();
extern VOID node_tbl_del(IN UINT8 u8_node_id);
extern VOID l1cc_dsp2arm_msg_send(UINT32 u32_msg_id, UINT16 u16_msg_len, UINT16 u16_mb_id, UINT32_PTR u32p_hls_msg_body);
#endif
/***************************************************** END OF FILE ****************************************************/

