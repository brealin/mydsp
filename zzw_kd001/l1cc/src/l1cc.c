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
#undef THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID L1CC_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "zzw_common.h"
#include "l1cc.h"
#include "ks_hs_drv.h"
#include "ks_phych_func.h"
#include "ks_kd001_main.h"
#define OAL_SRAM_INTERFACE_SECTION __attribute__((section(".DSECT OAL_SRAM_INTERFACE_SECTION")))

#ifdef CORE_X1643
#include "ofdm_tx_proc.h"
#else
#include "ofdm_rx_proc.h"
#endif

/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
OAL_DTCM_DATA_SECTION SINT32 g_s32_sleep_protect_slot = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_tmr_sav = 0;
OAL_DTCM_DATA_SECTION UINT32 g_u32_tmr_sav2 = 0;
#ifdef CORE_X1643
OAL_DTCM_DATA_SECTION ZZW_RTC_DSP_T g_zzw_rtc = {0,{0,0}};
OAL_DTCM_DATA_SECTION UINT8 g_zzw_slot_table[1024] = {0};
OAL_DTCM_DATA_SECTION OAL_ALIGNED(4) UINT8 g_zzw_sf_b[1134] = {0};
OAL_DTCM_DATA_SECTION ZZW_NODE_STATE_T g_zzw_node_status = {0};
OAL_DTCM_DATA_SECTION BOOLEAN  g_tx_pow_set_flag   = 0;
OAL_DTCM_DATA_SECTION BOOLEAN  g_rx_gain_set_flag  = 0;
OAL_DTCM_DATA_SECTION UINT32 g_zzw_mcs = 7;
OAL_CP_SHIFT_DATA_SECTION UINT16 g_zzw_cp_shift = 0;
OAL_DTCM_DATA_SECTION STATIC UINT32 s_u32_modu_pingpong_index = 0;
OAL_DTCM_DATA_SECTION SINT32 g_s32_state_change = 5;	//0;
OAL_DTCM_DATA_SECTION SINT16 g_zzw_tx_pwr   = 0;
OAL_DTCM_DATA_SECTION UINT16 g_zzw_rx_gain  = 0;
OAL_DTCM_DATA_SECTION UINT32 g_zzw_freq_val = 0;
OAL_DTCM_DATA_SECTION UINT32 g_zzw_freq_val_dl = 0;
OAL_DTCM_DATA_SECTION UINT32 g_zzw_freq_set_flag = 0;
OAL_DTCM_DATA_SECTION UINT16 g_zzw_freq_scan_flag  = 0;
OAL_DTCM_DATA_SECTION UINT16 g_zzw_freq_scan_star  = 0;
OAL_DTCM_DATA_SECTION UINT16 g_zzw_freq_scan_end   = 0;
OAL_DTCM_DATA_SECTION UINT32 g_zzw_ssap = 0;
OAL_DTCM_DATA_SECTION UINT8 g_u8_node_id_target = 0;
OAL_DRAM_DATA_SECTION mcs_tbl_des_t g_st_mcs_des_tbl = {0};
#endif
OAL_SRAM_INTERFACE_SECTION node_info_tbl_t s_st_node_info_tbl = {0};
OAL_SRAM_INTERFACE_SECTION WAVEFORM_OFDM_CFG g_st_waveform_cfg = {0};
/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/
#ifdef CORE_X1643
//extern complex_s16_t g_st_modu_out[2][37*512];

//extern ul_proc_link_t g_st_ul_proc_link;
extern l1cc2algo_ctrl_info_t g_st_ctrl_info[];
//extern encode_info_t g_st_data_enc_info_table[3][3];

//extern UINT8 g_u8_tx_buffer[TX_BUFFER_SIZE];
extern UINT32 g_u32_tx_buffer_read_offset;
extern UINT32 g_u32_tx_buffer_write_offset;

//extern UINT8 g_u8_modu_in_buffer[TX_BUFFER_SIZE>>2];
extern UINT32 g_u32_modu_in_read_offset;
extern UINT32 g_u32_modu_in_write_offset;
extern complex_s16_t* g_stp_modu_out_addr;

extern pkg_head_t g_pkg_head;
//extern UINT8 g_u8_tx_buffer[TX_BUFFER_SIZE];
extern UINT8 g_u8_test_flag;
extern SINT32 g_s32_frame_no;

extern UINT8 g_u8_tx_ctrl_info_cnt_in;
extern UINT8 g_u8_tx_ctrl_info_cnt_out;

extern UINT32 g_u32_tx_encode_cancel;

#endif

OAL_DTCM_DATA_SECTION mcs_tbl_t g_st_mcs_tbl[ZZW_MAX_MCS_NUM] = 
#ifdef MIMO2
{
/*descrition:   encode input len;           encode output len;              mod tpye;   rd_mod; cb_num;reserved;encode cb len; encode cb output len*/ 
/*              (without crc)               														            (with cb crc)					   */
/*MCS0*/  		{2432 - 24,        		    1200 * DATA_SYM_NUM, 			BPSK, 		3,		1,		1,		2432,		SC_NUM * DATA_SYM_NUM},
/*MCS1*/  		{3584 - 24,            		1200 * DATA_SYM_NUM, 			BPSK, 		3,		1,		1,		3584,		SC_NUM * DATA_SYM_NUM},
/*MCS2*/  		{5440 - 24,            		1200 * DATA_SYM_NUM, 			BPSK, 		3,		1,		1,		5440,		SC_NUM * DATA_SYM_NUM},
/*MCS3*/  	  	{(3584 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM * 2,  		QPSK,		0,		2,		1,		3584,		SC_NUM * DATA_SYM_NUM},
/*MCS4*/  		{(5184 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM * 2,		QPSK,		0,		2,		1,		5184,		SC_NUM * DATA_SYM_NUM}, 
/*MCS5*/  		{(5952 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM * 2,  		QPSK,		0,		2,		1,		5952,		SC_NUM * DATA_SYM_NUM},
/*MCS6*/  		{(5440 - 24) * 3 - 24, 		1200 * DATA_SYM_NUM * 4,   	    QAM16,		1,		3,		1,		5440,		SC_NUM * DATA_SYM_NUM * 4 / 3},
/*MCS7*/  		{(5184 - 24) * 4 - 24, 		1200 * DATA_SYM_NUM * 4, 		QAM16,		1,		4,		1,		5184,		SC_NUM * DATA_SYM_NUM},
/*MCS8*/  		{(6144 - 24) * 4 - 24, 		1200 * DATA_SYM_NUM * 4, 		QAM16,		1,		4,		1,		6144,		SC_NUM * DATA_SYM_NUM},
/*MCS9*/  		{(5184 - 24) * 5 - 24, 		1200 * DATA_SYM_NUM * 6,   	    QAM64,		2,		5,		1,		5184,		SC_NUM * DATA_SYM_NUM * 6 / 5},
/*MCS10*/  		{(5184 - 24) * 6 - 24, 		1200 * DATA_SYM_NUM * 6, 		QAM64,		2,		6,		1,		5184,		SC_NUM * DATA_SYM_NUM},
/*MCS11*/  		{(6144 - 24) * 6 - 24, 		1200 * DATA_SYM_NUM * 6, 		QAM64,		2,		6,		1,		6144,		SC_NUM * DATA_SYM_NUM},
/*MCS12*/  		{(5440 - 24) * 8 - 24, 		1200 * DATA_SYM_NUM * 8, 		QAM256,		3,		8,		1,		5440,		SC_NUM * DATA_SYM_NUM},
/*MCS13*/		{(5952 - 24) * 8 - 24,      1200 * DATA_SYM_NUM * 8,		QAM256,		3,		8,		1,		5952,		SC_NUM * DATA_SYM_NUM},
};
#else
{
/*descrition:   encode input len;           encode output len;  mod tpye;   rd_mod; cb_num;reserved;encode cb len; encode cb output len*/ 
/*              (without crc)               														(with cb crc)					   */
/*MCS0*/  		{3584 - 24,            		1200 * DATA_SYM_NUM, 			BPSK, 		3,		1,		1,		3584,		SC_NUM * DATA_SYM_NUM},
/*MCS0*/  		{5440 - 24,            		1200 * DATA_SYM_NUM, 			BPSK, 		3,		1,		1,		5440,		SC_NUM * DATA_SYM_NUM},
/*MCS1*/  	  	{(3584 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM,  			BPSK,		3,		2,		1,		3584,		SC_NUM * DATA_SYM_NUM / 2},
/*MCS2*/  	  	{(4096 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM,  			BPSK,		3,		2,		1,		4096,		SC_NUM * DATA_SYM_NUM / 2},
/*MCS3*/  		{(5184 - 24) * 2 - 24, 		1200 * DATA_SYM_NUM * 2,		QPSK,		0,		2,		1,		5184,		SC_NUM * DATA_SYM_NUM}, 
/*MCS4*/  		{(4352 - 24) * 3 - 24, 		1200 * DATA_SYM_NUM * 2,  		QPSK,		0,		3,		1,		4352,		SC_NUM * DATA_SYM_NUM * 2 / 3},
/*MCS5*/  		{(5440 - 24) * 3 - 24, 		1200 * DATA_SYM_NUM * 2,   	    QPSK,		0,		3,		1,		5440,		SC_NUM * DATA_SYM_NUM * 2 / 3},
/*MCS6*/  		{(5184 - 24) * 4 - 24, 		1200 * DATA_SYM_NUM * 4, 		QAM16,		1,		4,		1,		5184,		SC_NUM * DATA_SYM_NUM},
/*MCS7*/  		{(6144 - 24) * 4 - 24, 		1200 * DATA_SYM_NUM * 4, 		QAM16,		1,		4,		1,		6144,		SC_NUM * DATA_SYM_NUM},
/*MCS8*/  		{(5440 - 24) * 5 - 24, 		1200 * DATA_SYM_NUM * 4,   	    QAM16,		1,		5,		1,		5440,		SC_NUM * DATA_SYM_NUM * 4 / 5},
/*MCS9*/  		{(5184 - 24) * 6 - 24, 		1200 * DATA_SYM_NUM * 6, 		QAM64,		2,		6,		1,		5184,		SC_NUM * DATA_SYM_NUM},
/*MCS10*/  		{(6144 - 24) * 6 - 24, 		1200 * DATA_SYM_NUM * 6, 		QAM64,		2,		6,		1,		6144,		SC_NUM * DATA_SYM_NUM},
/*MCS11*/  		{(5440 - 24) * 8 - 24, 		1200 * DATA_SYM_NUM * 6, 		QAM64,		2,		8,		1,		5440,		SC_NUM * DATA_SYM_NUM * 6 / 8},
/*MCS12*/		{(6016 - 24) * 8 - 24,      1200 * DATA_SYM_NUM * 6,		QAM64,		2,		8,		1,		6016,		SC_NUM * DATA_SYM_NUM * 6 / 8},
/*MCS13*/		{(5952 - 24) * 9 - 24,      1200 * DATA_SYM_NUM * 6,		QAM64,		2,		9,		1,		5952,		SC_NUM * DATA_SYM_NUM * 6 / 9}
};
#endif
/***********************************************************************************************************************
* FUNCTION
*
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
UINT8 node_slot_alloc_query(IN UINT8 u8_ssap, IN UINT32 u32_fn)
{
	/*固定时隙配置时，每一个SLOT被固定分配给每个节点，所以可以通过帧号和时隙分配来查询当前帧对应的TX节点号*/
	UINT8 u8_ssap_local = u8_ssap;
	UINT8 u8_node_id = 0;
	UINT8 u8_bmp = 0;

	if(u8_ssap_local EQ 15)
	{
		/*
			固定时隙配置，15 TCP全收全发,	仅支持两个节点
			节点一：{0x35555555,0x55555555,0x35555555,0x55555555,0x35555555,0x55555555,0x35555555,0x55555555},
			节点二：{0x53333333,0x33333333,0x53333333,0x33333333,0x53333333,0x33333333,0x53333333,0x33333333},

			广播帧号为64的倍数：
			64的偶数倍，分配给节点1; 64的奇数倍，分配给节点2
		*/
		if ((u32_fn & 0xF) EQ 7)
		{
			//数据帧
			u8_node_id = 2;
		}
		else
		{
			if((u32_fn & 0x3F))
			{
				//数据帧
				u8_node_id = 1;
			}
			else
			{
				//广播帧
				u8_node_id = ((u32_fn >> 6) & 1) + 1;			
			}
		}		
	}
	else if(u8_ssap_local EQ 16)
	{
		/*
			固定时隙配置，16 UDP全收全发,	仅支持两个节点
			节点一：{0x35555555,0x55555555,0x55555555,0x55555555,0x35555555,0x55555555,0x55555555,0x55555555},
			节点二：{0x53333333,0x33333333,0x33333333,0x33333333,0x53333333,0x33333333,0x33333333,0x33333333},

			广播帧号为64的倍数：
			64的偶数倍，分配给节点1; 64的奇数倍，分配给节点2

		*/
		if ((u32_fn & 0x1F) EQ 7)
		{
			//数据帧
			u8_node_id = 2;
		}
		else
		{
			if((u32_fn & 0x3F))
			{
				//数据帧
				u8_node_id = 1;
			}
			else
			{
				//广播帧
				//u8_node_id = ((u32_fn >> 6) & 1) + 1;
				u8_node_id = 1;
				if((u32_fn & 0x7FF) EQ 64)
				{
					u8_node_id = 2;
				}
			}
		}
	}
	else if((u8_ssap_local EQ 17) || (u8_ssap_local EQ 18))
	{
		/*
			SSAP = 17, 二分之一时隙分配，
			数据帧号模2：
			余数为0，分配给节点2， 余数为1，分配给节点1
			广播帧号为64的倍数：
			64的偶数倍，分配给节点1; 64的奇数倍，分配给节点2

			SSAP = 18, 四分之一时隙分配，
			数据帧号模4：
			余数为0，分配给节点4， 余数为1，分配给节点3， 余数为2，分配给节点2， 余数为3，分配给节点1
			广播帧号为64的倍数：
			64 * (4 * N)，分配给节点1; 64 * (4 * N + 1)，分配给节点2; 64 * (4 * N + 2)，分配给节点3; 64 * (4 * N + 3)，分配给节点4;
			

		*/
		u8_ssap_local -= 16;
		u8_bmp = (1 << u8_ssap_local);
		if(u32_fn & 0x3F)
		{
			//数据帧
			u8_node_id = u8_bmp - (u32_fn & (u8_bmp - 1));
		}
		else
		{
			//广播帧
			u8_node_id = ((u32_fn >> 6) & (u8_bmp - 1)) + 1;
		}
	}
	else if(u8_ssap_local EQ 19)
	{
		//to be done
		#if 1
		/*
			SSAP = 19, 五分之一时隙分配，
			节点1：{0x33353333,0x35333353,0x33335333,0x33533335,0x53333533,0x33353333,0x35333353,0x33335333},
			节点2：{0x33335333,0x33533335,0x53333533,0x33353333,0x35333353,0x33335333,0x33533335,0x53333533},
	        节点3：{0x53333533,0x33353333,0x35333353,0x33335333,0x33533335,0x53333533,0x33353333,0x35333353},
	        节点4：{0x35333353,0x33335333,0x33533335,0x53333533,0x33353333,0x35333353,0x33335333,0x33533335},
	        节点5：{0x33533335,0x53333533,0x33353333,0x35333353,0x33335333,0x33533335,0x53333533,0x33353333},
		
			数据帧号先模64，余数再模5：
			余数为0，分配给节点5， 余数为1，分配给节点4
			广播帧号先模2048，余数为64的倍数：
			64 * (5 * N)，分配给节点1; 64 * (5 * N + 1)，分配给节点2; 64 * (5 * N + 2)，分配给节点3; 64 * (5 * N + 3)，分配给节点4;
		*/
		if(u32_fn & 0x3F)
		{
			//数据帧
			u8_node_id = 5 - ((u32_fn & (0x3F)) % 5);
		}
		else
		{
			//广播帧
			u8_node_id = (((u32_fn & (0x7FF)) >> 6) % 5) + 1;
		}
		#endif
	}
	else if((u8_ssap_local >= 20) && (u8_ssap_local <=22))
	{
		/*
			SSAP = 20, 八分之一时隙分配，
			数据帧号模8：
			余数为0，分配给节点8， 余数为1，分配给节点7， 余数为2，分配给节点6， 余数为3，分配给节点5
			余数为4，分配给节点4， 余数为5，分配给节点3， 余数为6，分配给节点2， 余数为7，分配给节点1

			广播帧号为64的倍数：
			64 * (8 * N)，分配给节点1; 64 * (8 * N + 1)，分配给节点2; 64 * (8 * N + 2)，分配给节点3; 64 * (8 * N + 3)，分配给节点3;
			64 * (8 * N + 4)，分配给节点4; 64 * (8 * N + 5)，分配给节点5; 64 * (8 * N + 6)，分配给节点6; 64 * (8 * N + 7)，分配给节点7;
			十六分之一与三十二分之一逻辑类似，以此类推。。
		*/
		u8_ssap_local -= 17;
		u8_bmp = (1 << u8_ssap_local);
		if(u32_fn & 0x3F)
		{
			//数据帧
			u8_node_id = u8_bmp - (u32_fn & (u8_bmp - 1));
		}
		else
		{
			//广播帧
			u8_node_id = ((u32_fn >> 6) & (u8_bmp - 1)) + 1;
		}
	}
	else
	{
		//to be done
	}
	KS_LOG(0, 0xA162, u8_node_id);

	return u8_node_id;
}

OAL_ITCM_CODE_SECTION
VOID node_tbl_add(IN node_info_t *stp_node_info_in)
{
	UINT16 u16_node_num;
	UINT32 u32_i = 0;
	node_info_tbl_t *stp_node_info_tbl = &s_st_node_info_tbl;
	node_info_t *stp_node_info = s_st_node_info_tbl.st_node_info;

	KS_LOG(0, 0xA160, stp_node_info_in->u8_node_id);
	if(stp_node_info_tbl->u8_is_valid)
	{
		u16_node_num = stp_node_info_tbl->u8_node_num;
		OAL_ASSERT(u16_node_num <= (MAX_NODE_NUM), "");

		for(u32_i = 0; u32_i < u16_node_num; u32_i++)
		{
			//reorder by node id with increase order
			if(stp_node_info->u8_node_id > stp_node_info_in->u8_node_id)
			{
				OAL_ASSERT(u16_node_num <= (MAX_NODE_NUM - 1), "");
				if(u16_node_num - u32_i > 0)
				{
					ks_oal_mem_copy((VOID_PTR)(stp_node_info + 1), (VOID_PTR)(stp_node_info), (u16_node_num - u32_i) * sizeof(node_info_t));
				}
				ks_oal_mem_copy((VOID_PTR)(stp_node_info), (VOID_PTR)(stp_node_info_in), sizeof(node_info_t));
				stp_node_info_tbl->u8_node_num += 1;
				break;
			}
			else if(stp_node_info->u8_node_id EQ stp_node_info_in->u8_node_id)
			{
				//node already exists, to be done
				OAL_ASSERT(0, "");
				break;
			}
			stp_node_info += 1;
		}

		if(u16_node_num EQ stp_node_info_tbl->u8_node_num)
		{
			//add new node to the end 
			ks_oal_mem_copy((VOID_PTR)(stp_node_info), (VOID_PTR)(stp_node_info_in), sizeof(node_info_t));
			stp_node_info_tbl->u8_node_num += 1;
		}
	}
	else
	{
		stp_node_info_tbl->u8_is_valid = OAL_TRUE;
		stp_node_info_tbl->u8_node_num = 1;

		ks_oal_mem_copy((VOID_PTR)(stp_node_info), (VOID_PTR)(stp_node_info_in), sizeof(node_info_t));

	}
}
OAL_ITCM_CODE_SECTION
VOID node_tbl_del(IN UINT8 u8_node_id)
{
	UINT16 u16_node_num;
	UINT32 u32_i = 0;
	node_info_tbl_t *stp_node_info_tbl = &s_st_node_info_tbl;
	node_info_t *stp_node_info = s_st_node_info_tbl.st_node_info;

	KS_LOG(0, 0xA161, u8_node_id);
	if(stp_node_info_tbl->u8_is_valid)
	{
		u16_node_num = stp_node_info_tbl->u8_node_num;
		OAL_ASSERT(u16_node_num <= (MAX_NODE_NUM), "");

		for(u32_i = 0; u32_i < u16_node_num; u32_i++)
		{
			if(stp_node_info->u8_node_id EQ u8_node_id)
			{
				OAL_ASSERT(u16_node_num <= (MAX_NODE_NUM - 1), "");
				if(u16_node_num - u32_i > 1)
				{
					ks_oal_mem_copy((VOID_PTR)(stp_node_info), (VOID_PTR)(stp_node_info + 1), (u16_node_num - u32_i - 1) * sizeof(node_info_t));
				}
				stp_node_info_tbl->u8_node_num -= 1;
				break;
			}
			stp_node_info += 1;
		}

		if(stp_node_info_tbl->u8_node_num EQ 0)
		{
			stp_node_info_tbl->u8_is_valid = OAL_FALSE;
			ks_oal_mem_set((VOID_PTR)(stp_node_info), 0, sizeof(node_info_t));

		}
		KS_LOG(0, 0xA161, stp_node_info_tbl->u8_node_num);
	}
}

OAL_ITCM_CODE_SECTION
VOID_PTR node_tbl_find(IN UINT8 u8_node_id)
{
	UINT16 u16_node_num;
	UINT32 u32_i = 0;
	node_info_tbl_t *stp_node_info_tbl = &s_st_node_info_tbl;
	node_info_t *stp_node_info = s_st_node_info_tbl.st_node_info;
	VOID_PTR vp_ret = NULL_PTR;

	if(u8_node_id EQ 0)
	{
		return NULL_PTR;			
	}
	if(stp_node_info_tbl->u8_is_valid)
	{
		u16_node_num = stp_node_info_tbl->u8_node_num;
		OAL_ASSERT(u16_node_num <= (MAX_NODE_NUM), "");

		for(u32_i = 0; u32_i < u16_node_num; u32_i++)
		{
			if(stp_node_info->u8_node_id EQ u8_node_id)
			{
				//node already exists, to be done
				vp_ret = (VOID_PTR)(stp_node_info);
				break;
			}
			stp_node_info += 1;
		}
		return vp_ret;
	}
	else
	{
		return NULL_PTR;			
	}
}
OAL_ITCM_CODE_SECTION
VOID node_tbl_clear()
{
	node_info_tbl_t *stp_node_info_tbl = &s_st_node_info_tbl;

	stp_node_info_tbl->u8_is_valid = OAL_FALSE;
	#ifdef FUN_OTD
	stp_node_info_tbl->s16_otd = 0;
	stp_node_info_tbl->u8_otd_adj_flg = OAL_FALSE;
	stp_node_info_tbl->u32_ref_bmp = 0;
	#endif
}
OAL_ITCM_CODE_SECTION
VOID l1cc_dsp2arm_msg_send(UINT32 u32_msg_id, UINT16 u16_msg_len, UINT16 u16_mb_id, UINT32_PTR u32p_hls_msg_body)
{
    UINT16 msg_len = ((UINT16)sizeof(oal_hls_msg_t) + u16_msg_len);

    oal_msg_t *stp_msg;
    oal_hls_msg_t* stp_hls_msg;

    stp_msg = OAL_MSG_CREAT(msg_len>>1);
    OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

    stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
    stp_msg->u16_mode_type    = 3; //FLIGHT_MODE;
    stp_msg->u16_standby_id   = 0; //STANDBY_0;

    stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
    stp_hls_msg->u16_msg_size   = msg_len;
    stp_hls_msg->b8_src_id      = 10;
    stp_hls_msg->u16_pdu_size   = 0;
    stp_hls_msg->u8_phy_task_id = 2;
    stp_hls_msg->b8_standby_id  = 0; //STANDBY_0
    stp_hls_msg->b8_mode_type   = 3; //FLIGHT_MODE;

    stp_hls_msg->u32_msg_id = u32_msg_id; //X1643 P_L1_HLS_SWITCHON_IND;
    if(u16_msg_len)
    {
		ks_oal_mem_copy((VOID_PTR)L1CC_GET_HLS_MSG_BODY(stp_hls_msg), (VOID_PTR)u32p_hls_msg_body, u16_msg_len);
	}
    ks_oal_dsp2arm_msg_send(stp_msg, NULL_PTR, 0, u16_mb_id, (UINT16)OAL_SEND_IMMEDIATE, (UINT16)OAL_IPC_WAIT_FOREVER);
}


OAL_ITCM_CODE_SECTION
VOID l1cc_sys_main(VOID)
{
    VOLATILE_UINT32_PTR vu32_ptr;
    VOLATILE_UINT32_PTR vu32_cp_ptr;
    //VOLATILE_UINT32_PTR vu32_cpu_status_ptr;
	UINT32 u32_tmr_start = 0;
	UINT32 u32_tmr_end = 0;
	UINT32 u32_i = 0;
	VUINT32 vu32_cp_heart_beat;
	OAL_IRQ_SAVE_AREA;
	

	OAL_IRQ_DISABLE_ALL;
	u32_tmr_start = ks_timer_get_counter(KS_CP0_TIMER2);
    ks_oal_delayus(1000);/*1ms*/
	u32_tmr_end = ks_timer_get_counter(KS_CP0_TIMER2);
	g_u32_tmr_sav = (u32_tmr_start - u32_tmr_end);
	OAL_IRQ_RESTORE;

	#if 0
	OAL_IRQ_DISABLE_ALL;
	u32_tmr_start = ks_timer_get_counter(KS_CP0_TIMER2);
	for(u32_i = 0; u32_i < 500; u32_i++)
	{
		KS_LOG(0, 0xA111, (UINT16)u32_i);
	}
	u32_tmr_end = ks_timer_get_counter(KS_CP0_TIMER2);
	g_u32_tmr_sav2 = (u32_tmr_start - u32_tmr_end);
	OAL_IRQ_RESTORE;
	#endif


#ifdef CORE_X1643
	#ifdef TX_TEST
	g_u32_extend_ini_flg = 1;
	zzw_waveform_init();
    ks_oal_drv_init_1643();
    #endif

    if(cp1_x1643_def_flag == 1)
    {            
        vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP1_X1643) + 0x400000);
        vu32_cp_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CPCPU1) + 0x400000);
    	//vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP1_X1643));
    }
    else
    {
        vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP0_X1643));
        vu32_cp_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CPCPU0));
    	//vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP0_X1643));        
    }
    #ifndef TX_TEST
    vu32_cp_heart_beat = READ_REG32(vu32_cp_ptr);
	while(vu32_cp_heart_beat EQ READ_REG32(vu32_cp_ptr))
	{
        KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , (UINT16)vu32_cp_heart_beat);
        //KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , ((UINT32)vu32_cp_ptr)>>16);
		ks_oal_delay((UINT32)20000);/*1ms*/
	}
	l1cc_dsp2arm_msg_send(60538, 0, (UINT16)OAL_L1_HLS_WR_MB0_ID, NULL_PTR);
	#endif
    //rf_ctrl_enable();
#else
    if(cp1_xc4500_def_flag == 1)
    {            
        vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP1_XC4500) + 0x400000);
        vu32_cp_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CPCPU1) + 0x400000);
    	//vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP1_XC4500));        
    }
    else
    {
        vu32_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CP0_XC4500)); 
        vu32_cp_ptr = (VOLATILE_UINT32_PTR)(HEARTBEAT_ADDR(KS_CPU_CPCPU0));
    	//vu32_cpu_status_ptr = (VOLATILE_UINT32_PTR)(KS_SYS_STATUS_ADDR(KS_CPU_CP0_XC4500));    
    }
    #ifndef TX_TEST
    vu32_cp_heart_beat = READ_REG32(vu32_cp_ptr);
	while(vu32_cp_heart_beat EQ READ_REG32(vu32_cp_ptr))
	{
        KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , (UINT16)vu32_cp_heart_beat);
        //KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , ((UINT32)vu32_cp_ptr)>>16);
		ks_oal_delay((UINT32)20000);/*1ms*/
	}
	l1cc_dsp2arm_msg_send(60540, 0, (UINT16)OAL_L1_HLS_WR_MB3_ID, NULL_PTR);
	#endif
    //dl_proc_main();

#endif
	*vu32_ptr = 0;
    while(1)
	{
		#if 1
		u32_tmr_start = ks_timer_get_counter(KS_CP0_TIMER2);
        *vu32_ptr += 1;
        ks_oal_delayus(1000);/*1ms*/
		u32_tmr_end = ks_timer_get_counter(KS_CP0_TIMER2);
		u32_tmr_start -= u32_tmr_end;
		//OAL_ASSERT((u32_tmr_start NEQ 0), "");
        KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , (UINT16)(1000 * g_u32_tmr_sav / u32_tmr_start));
        #else
        //OAL_IRQ_DISABLE;
		u32_tmr_start = ks_timer_get_counter(KS_CP0_TIMER2);
		
		//OAL_IRQ_DISABLE_ALL;

		if(DSP_DEEPSLEEP_PRE == g_enum_dsp_state)
		{
			ks_slp_enter_final();
		}


        //OAL_IRQ_RESTORE;
		#ifdef CORE_X1643
    	_light_sleep();
		#else
    	_low_power_4500();
    	#endif
		
		u32_tmr_end = ks_timer_get_counter(KS_CP0_TIMER2);
		u32_tmr_start -= u32_tmr_end;
		//OAL_ASSERT((u32_tmr_start NEQ 0), "");
        KS_LOG(OAL_TRACE_LEVEL_0 , 0xA100 , (UINT16)(1000 * g_u32_tmr_sav / u32_tmr_start));
        *vu32_ptr += 1;
        #endif
	};
}

/***********************************************************************************************************************
* FUNCTION
*
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
#ifdef CORE_XC4500
extern VOID dl_data_dec_rpt();
#endif

OAL_ITCM_CODE_SECTION
VOID ks_oal_dispatch_proc(VOID)
{
#ifdef CORE_X1643
	ks_oal_arm2dsp_msg_read((CONST_UINT16)OAL_HLS_L1_RD_MB0_ID);
	// ks_oal_arm2dsp_msg_read((CONST_UINT16)OAL_HLS_L1_RD_MB2_ID);
	ks_oal_inter_dsp_msg_read();
#else
	dl_data_dec_rpt();
#endif

	//ks_oal_inter_dsp_msg_read();

#if defined(CORE_X1643)
    //*(unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~~(unsigned int)(0x1 << OAL_IRQ_CP_MAILBOX_X1643_OUT); //unmask ictl int
#else defined(CORE_XC4500)
    //*(unsigned int *)(ICTL0_IRQ_INTMASK_L) &= ~~(unsigned int)(0x1 << OAL_IRQ_CP_MAILBOX_XC4500_OUT); //unmask ictl int
#endif

}

/***********************************************************************************************************************
* FUNCTION
*
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
//@Zhemin
OAL_ITCM_CODE_SECTION
VOID l1cc_dispatch_main(VOID)
{
    while(1)
    {
        if(OAL_SUCCESS == ks_oal_sema_get(OAL_L1CC_IPC_SEMA, OAL_IPC_WAIT_FOREVER))
        {
            ks_oal_dispatch_proc();
        }
    }
}


/***********************************************************************************************************************
* FUNCTION
*
* VOID l1cc_hl_msg_proc(VOID)
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
#ifdef CORE_X1643
VOLATILE_UINT32 u32_test_counter[4] = {0,0,0,0};

OAL_ITCM_CODE_SECTION VOID zzw_waveform_init()
{
	g_st_waveform_cfg.u32_samp_rate = SAMPLE_RATE;
	g_st_waveform_cfg.u32_frame_len = FRAME_LEN;
	g_st_waveform_cfg.u16_cp_len = CP_LEN;
	g_st_waveform_cfg.u16_symbol_len = SYMBOL_LEN;
	g_st_waveform_cfg.u16_gp0_len = GAP0_LEN;
	g_st_waveform_cfg.u16_gp1_len = GAP1_LEN;
	g_st_waveform_cfg.u16_agc_len = AGC_LEN;
	g_st_waveform_cfg.u16_cca_len = CCA_LEN;
	g_st_waveform_cfg.u8_data_sym_num = DATA_SYM_NUM;
	g_st_waveform_cfg.u8_total_frame_seg = TOTAL_FRAME_SEG_NUM;

	g_st_waveform_cfg.u16_frame_seg_len[GAP0_TYPE] = GAP0_LEN;
	g_st_waveform_cfg.u16_frame_seg_len[AGC_CCA_TYPE] = (AGC_LEN + (CCA_LEN << 1));
	g_st_waveform_cfg.u16_frame_seg_len[PILOT_SYM_TYPE] = SYMBOL_LEN;//SLOT_LEN;
	g_st_waveform_cfg.u16_frame_seg_len[DATA_SYM_TYPE] = SYMBOL_LEN;
	g_st_waveform_cfg.u16_frame_seg_len[GAP1_TYPE] = GAP1_LEN;

	g_st_waveform_cfg.u32_frame_seg_map[0] = FRAME_SEG_MAP0;
	g_st_waveform_cfg.u32_frame_seg_map[1] = FRAME_SEG_MAP1;
}

OAL_ITCM_CODE_SECTION VOID zzw_mcs_tbl_init()
{
	UINT32 u32_i;
    UINT32 u32_msg_id;
    UINT16 u16_msg_len;
    VOID_PTR vp_msg_body = NULL_PTR;
	THls_Phy_buf_info* mbuf_info;
    oal_msg_t *stp_msg;
    oal_hls_msg_t* stp_hls_msg;
    UINT32_PTR u32p_temp = NULL_PTR;
    UINT16 msg_len;		
	SINT8 s8_snr_thres[ZZW_MAX_MCS_NUM][2] = {{-100, -3}, {-3, 0}, {0, 3}, {3, 6}, \
											{6, 9}, {9, 10}, {10, 13}, {13, 13}, \
											{13, 17}, {17, 21}, {21, 21}, {21, 25},\
											{25, 27}, {27, 100}};
											
	g_st_mcs_des_tbl.u8_mcs_num = ZZW_MAX_MCS_NUM;

	for(u32_i = 0; u32_i < ZZW_MAX_MCS_NUM; u32_i++)
	{
		g_st_mcs_des_tbl.st_mcs_des[u32_i].u8_mcs_lvl = u32_i;			
		g_st_mcs_des_tbl.st_mcs_des[u32_i].u16_payload_len = (g_st_mcs_tbl[u32_i].u32_raw_len >> 3);
		g_st_mcs_des_tbl.st_mcs_des[u32_i].s8_lower_limit = s8_snr_thres[u32_i][0];
		g_st_mcs_des_tbl.st_mcs_des[u32_i].s8_upper_limit = s8_snr_thres[u32_i][1];		
	}

	#if 0
    msg_len = sizeof(oal_hls_msg_t) + sizeof(mcs_tbl_des_t);

    stp_msg = ks_oal_msg_create(msg_len>>1);
    OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

    stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
    stp_msg->u16_mode_type    = 3; //FLIGHT_MODE;
    stp_msg->u16_standby_id   = 0; //STANDBY_0;

    stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
    stp_hls_msg->u16_msg_size   = msg_len;
    stp_hls_msg->b8_src_id      = 10;
    stp_hls_msg->u16_pdu_size   = 0;
    stp_hls_msg->u8_phy_task_id = 2;
    stp_hls_msg->b8_standby_id  = 0; //STANDBY_0
    stp_hls_msg->b8_mode_type   = 3; //FLIGHT_MODE;
    stp_hls_msg->u32_msg_id = 60550;

    u32p_temp = (UINT32_PTR)&(stp_hls_msg->vp_msg_body);
	ks_oal_mem_copy(u32p_temp, &g_st_mcs_des_tbl, sizeof(mcs_tbl_des_t));
    ks_oal_dsp2arm_msg_send(stp_msg, NULL_PTR, 0, (UINT16)OAL_L1_HLS_WR_MB1_ID, (UINT16)OAL_SEND_IMMEDIATE, (UINT16)OAL_IPC_WAIT_FOREVER);
    #else
	l1cc_dsp2arm_msg_send(60550, sizeof(mcs_tbl_des_t), (UINT16)OAL_L1_HLS_WR_MB1_ID,&g_st_mcs_des_tbl);

    #endif
	
}

OAL_ITCM_CODE_SECTION
VOID l1cc_hl_msg_proc(IN CONST oal_hls_msg_t * CONST stp_oal_hls_msg)
{
    UINT32 u32_msg_id;
    UINT16 u16_msg_len;
    VOID_PTR vp_msg_body = NULL_PTR;
	THls_Phy_buf_info* mbuf_info;
    oal_msg_t *stp_msg;
    oal_hls_msg_t* stp_hls_msg;
    UINT32_PTR u32p_temp = NULL_PTR;
    UINT16 msg_len;	
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);	//g_st_iq_dump

    OAL_IRQ_SAVE_AREA;
    u32_msg_id = stp_oal_hls_msg->u32_msg_id;
    u16_msg_len = (stp_oal_hls_msg->u16_msg_size - L1CC_HLS_MSG_HEAD_SIZE);
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA101 , (UINT16)u32_msg_id);
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xA102 , (UINT16)u16_msg_len);

	if(stp_oal_hls_msg->u16_pdu_size != 0)
    {
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xA103 , (UINT16)stp_oal_hls_msg->u16_pdu_size);
        OAL_ASSERT(stp_oal_hls_msg->u16_pdu_size == 4,"peer_len error!");
        mbuf_info = (THls_Phy_buf_info* )((uint8 *)L1CC_GET_HLS_MSG_BODY(stp_oal_hls_msg) + 4);
        // vp_msg_body = mbuf_info;// ks_oal_get_recv_buff_ptr(mbuf_info->buf_index);
        u32_test_counter[0]++;
    }
	else 
	{
		 vp_msg_body = (VOID_PTR)OAL_GET_OAL_MSG_BODY(stp_oal_hls_msg);
	}

    switch (u32_msg_id)
    {
        case ZZW_MACREQ_H2L_PACKAGE:
			u32_test_counter[1]++; 
			// *((UINT32_PTR)vp_msg_body) = 0x12345678;
            break;
			
        case ZZW_MACREQ_SET_RTC:
            OAL_IRQ_DISABLE;
			//OAL_ASSERT(g_zzw_rtc.u32_valid == OAL_FALSE, "set rtc confliction\n");
			if(!g_zzw_rtc.u32_valid)
			{
				ks_oal_mem_copy(&g_zzw_rtc.st_rtc, vp_msg_body, sizeof(ZZW_RTC_T));
	        	g_zzw_rtc.u32_valid = OAL_TRUE;
        	}
            OAL_IRQ_RESTORE;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA104 , (UINT16)g_zzw_rtc.st_rtc.s32_frame_no);
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA104 , (UINT16)g_zzw_rtc.st_rtc.s32_rtc_value);
            break;

        case ZZW_MACREQ_SET_SLOT_TABLE:
            OAL_IRQ_DISABLE;
            #ifndef TX_TEST
            if(0 == g_zzw_freq_scan_flag)
            {
                ks_oal_mem_copy(&g_zzw_slot_table[0], vp_msg_body, sizeof(g_zzw_slot_table));
                g_zzw_ssap = *(UINT32_PTR)(vp_msg_body + sizeof(g_zzw_slot_table));
            }
            #endif
            stp_sarm_1643_4500->u8_ssap = (UINT8)g_zzw_ssap;
            OAL_IRQ_RESTORE;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA105 , *(UINT16_PTR)g_zzw_slot_table);    
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA105 , (UINT16)g_zzw_ssap);            

            break;

        case ZZW_MACREQ_SET_SF_B:
            OAL_IRQ_DISABLE;
            ks_oal_mem_copy(&g_zzw_sf_b, vp_msg_body, sizeof(g_zzw_sf_b));
			OAL_ASSERT(u16_msg_len <= sizeof(g_zzw_sf_b), "");
            //ks_oal_mem_copy(&g_zzw_sf_b, vp_msg_body, u16_msg_len);
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA105 , (UINT16)(((UPLINK_PHY_HEADER_TYPE *)g_zzw_sf_b)->frame_len));            
            OAL_IRQ_RESTORE;
            break;

        case ZZW_MACREQ_SET_NODE_STATE:
            OAL_IRQ_DISABLE;
            g_s32_state_change = 5;
			#ifndef TX_TEST

            ks_oal_mem_copy(&g_zzw_node_status, vp_msg_body, sizeof(ZZW_NODE_STATE_T));
            if(!g_u32_extend_ini_flg)
            {
				g_u32_extend_ini_flg = 1;
				zzw_waveform_init();
            	ks_oal_drv_init_1643();
            }
            if(ZZW_STATE_SCN EQ g_zzw_node_status.state)
            {
                //g_u8_agc_gain_stor = g_u8_rssi_stat;
				//g_u8_agc_gain_lev  = g_u8_rssi_stat;
				#ifdef MIMO2
				zzw_mcs_tbl_init();
				#endif
                ks_oal_mem_set(&g_zzw_slot_table[0], 0x22, sizeof(g_zzw_slot_table));
            }
            #if 0
            else if(1 == g_zzw_node_status.lc_id)
            {
                UINT32 u32_iloop = 0;
                for(; u32_iloop < 1024; )
                {
                    g_zzw_slot_table[u32_iloop]     = 0x33;
                    g_zzw_slot_table[u32_iloop + 1] = 0x35;
                    g_zzw_slot_table[u32_iloop + 2] = 0x33;
                    g_zzw_slot_table[u32_iloop + 3] = 0x35;
                    u32_iloop += 4;
                }

                g_zzw_slot_table[0]  = 0x34;
                g_zzw_slot_table[32] = 0x32;
                //ks_oal_mem_set(&g_zzw_slot_table[0], 0x00, sizeof(g_zzw_slot_table));
            }
            else if(2 == g_zzw_node_status.lc_id)
            {
                UINT32 u32_iloop = 0;
                for(; u32_iloop < 1024; )
                {
                    g_zzw_slot_table[u32_iloop]     = 0x53;
                    g_zzw_slot_table[u32_iloop + 1] = 0x33;
                    g_zzw_slot_table[u32_iloop + 2] = 0x53;
                    g_zzw_slot_table[u32_iloop + 3] = 0x33;
                    u32_iloop += 4;
                }

                g_zzw_slot_table[0]  = 0x32;
                g_zzw_slot_table[32] = 0x34;
                //ks_oal_mem_set(&g_zzw_slot_table[0], 0x00, sizeof(g_zzw_slot_table));
            }
            #endif
            #endif
            OAL_IRQ_RESTORE;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA106 , (UINT16)g_zzw_node_status.state);
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA106 , (UINT16)g_zzw_node_status.lc_id);
            break;
		case ZZW_MACREQ_SET_RF_POWER:
			OAL_IRQ_DISABLE;
			#ifndef TX_TEST
            ks_oal_mem_copy(&g_zzw_tx_pwr, vp_msg_body, sizeof(SINT16));
			//g_tx_pow_set_flag = OAL_TRUE;
			#endif
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA107 , (UINT16)g_zzw_tx_pwr);
			OAL_IRQ_RESTORE;
            break;
		case ZZW_MACREQ_SET_RF_RX_GAIN:
			OAL_IRQ_DISABLE;
			#ifndef TX_TEST
            ks_oal_mem_copy(&g_zzw_rx_gain, vp_msg_body, sizeof(UINT16)); 
			#endif
			//g_rx_gain_set_flag = OAL_TRUE;
			OAL_IRQ_RESTORE;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA108 , (UINT16)g_zzw_rx_gain);
            break;
		case ZZW_MACREQ_SET_RF_RX_FREQ:
			OAL_IRQ_DISABLE;
			#ifndef TX_TEST
            if((OAL_FALSE == g_zzw_freq_set_flag) && (g_zzw_freq_val NEQ (*(UINT32_PTR)vp_msg_body)))
            {
            	//first 4 bytes for ul freq, last 4 bytes for dl freq. if last 4 bytes is zero, dl freq = ul freq.
                ks_oal_mem_copy(&g_zzw_freq_val, vp_msg_body, sizeof(UINT32));
                ks_oal_mem_copy(&g_zzw_freq_val_dl, vp_msg_body + 4, sizeof(UINT32));
                g_zzw_freq_set_flag = OAL_TRUE;
            }
			#endif
			OAL_IRQ_RESTORE;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA109 , (UINT16)(g_zzw_freq_val >> 16));
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA109 , (UINT16)g_zzw_freq_val);
            break;
 
        case ZZW_MACREQ_SET_CP_SHIFT:
            g_zzw_cp_shift = *(UINT16_PTR)vp_msg_body;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xA10A , (UINT16)g_zzw_cp_shift);
		    break;
        case ZZW_MACREQ_SET_FREQ_SCAN:
            g_zzw_freq_scan_flag = *(UINT16_PTR)vp_msg_body;
            if(1 == g_zzw_freq_scan_flag)
            {
                g_zzw_freq_scan_star = 1;
                ks_oal_mem_set(&g_zzw_slot_table[0], 0x22, sizeof(g_zzw_slot_table)); 
            }
            else
            {
                g_zzw_freq_scan_end = 1;
            }
            break;

        case ZZW_MACREQ_GET_BLER:
        	#if 0
            msg_len = sizeof(oal_hls_msg_t) + 16;

            stp_msg = ks_oal_msg_create(msg_len>>1);
            OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

            stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
            stp_msg->u16_mode_type    = 3; //FLIGHT_MODE;
            stp_msg->u16_standby_id   = 0; //STANDBY_0;

            stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
            stp_hls_msg->u16_msg_size   = msg_len;
            stp_hls_msg->b8_src_id      = 10;
            stp_hls_msg->u16_pdu_size   = 0;
            stp_hls_msg->u8_phy_task_id = 2;
            stp_hls_msg->b8_standby_id  = 0; //STANDBY_0
            stp_hls_msg->b8_mode_type   = 3; //FLIGHT_MODE;
            stp_hls_msg->u32_msg_id = ZZW_MACREQ_GET_BLER + 1;

            u32p_temp = (UINT32_PTR)&(stp_hls_msg->vp_msg_body);
            u32p_temp[0] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x14);
            u32p_temp[1] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x18);
            u32p_temp[2] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x1C);
            u32p_temp[3] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x20);

            ks_oal_mem_set((VOID_PTR)(RX_DEBUG_INFO_ADDR + 0x14), 0x00, 16);
            ks_oal_dsp2arm_msg_send(stp_msg, NULL_PTR, 0, (UINT16)OAL_L1_HLS_WR_MB1_ID, (UINT16)OAL_SEND_IMMEDIATE, (UINT16)OAL_IPC_WAIT_FOREVER);
            #else
			l1cc_dsp2arm_msg_send(ZZW_MACREQ_GET_BLER + 1, 16, (UINT16)OAL_L1_HLS_WR_MB1_ID, (UINT32_PTR)(RX_DEBUG_INFO_ADDR + 0x14));
            ks_oal_mem_set((VOID_PTR)(RX_DEBUG_INFO_ADDR + 0x14), 0x00, 16);
            #endif
            break;

        case ZZW_MACREQ_GET_CH_PARA:
        	#if 0
            msg_len = sizeof(oal_hls_msg_t) + 20;
            stp_msg = ks_oal_msg_create(msg_len>>1);
            OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

            stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
            stp_msg->u16_mode_type    = 3; //FLIGHT_MODE;
            stp_msg->u16_standby_id   = 0; //STANDBY_0;

            stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
            stp_hls_msg->u16_msg_size   = msg_len;
            stp_hls_msg->b8_src_id      = 10;
            stp_hls_msg->u16_pdu_size   = 0;
            stp_hls_msg->u8_phy_task_id = 2;
            stp_hls_msg->b8_standby_id  = 0; //STANDBY_0
            stp_hls_msg->b8_mode_type   = 3; //FLIGHT_MODE;
            stp_hls_msg->u32_msg_id = ZZW_MACREQ_GET_CH_PARA + 1;

	        u32p_temp = (UINT32_PTR)&(stp_hls_msg->vp_msg_body);
	        u32p_temp[0] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x58); // s32_cfo_value
	        u32p_temp[1] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x5C); // s32_rtc_time
	        u32p_temp[2] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x60); // s32_shift_value
	        u32p_temp[3] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x64); // s32_signal_pwr
	        u32p_temp[4] = READ_REG32(RX_DEBUG_INFO_ADDR + 0x68); // s32_noise_pwr

            ks_oal_mem_set((VOID_PTR)(RX_DEBUG_INFO_ADDR + 0x58), 0x00, 20);
			
	        ks_oal_dsp2arm_msg_send(stp_msg, NULL_PTR, 0, (UINT16)OAL_L1_HLS_WR_MB1_ID, (UINT16)OAL_SEND_IMMEDIATE, (UINT16)OAL_IPC_WAIT_FOREVER);
	        #else
			l1cc_dsp2arm_msg_send(ZZW_MACREQ_GET_CH_PARA + 1, 20, (UINT16)OAL_L1_HLS_WR_MB1_ID, (UINT32_PTR)(RX_DEBUG_INFO_ADDR + 0x58));
            ks_oal_mem_set((VOID_PTR)(RX_DEBUG_INFO_ADDR + 0x58), 0x00, 20);
	        #endif
	    	break;	
	    	
		default:
            break;
    }

    return;
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID l1cc_hl_proc_main(VOID)
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_ITCM_CODE_SECTION
VOID l1cc_hl_proc_main(VOID)
{
    oal_msg_t *stp_msg;
    oal_primitive_id_t CONST st_wait_msg_list[2] = {(UINT32)1, (UINT32)OAL_ANY_MSG_ID};
    oal_primitive_id_t st_primitive_id;
    CONST_VOID_PTR vp_oal_msg_body;

    while (1)
    {
        //OAL_TASK_SAFE_SET();
        /* Wait for one message forever */
        stp_msg = ks_oal_msg_receive(st_wait_msg_list, (UINT16)OAL_IPC_WAIT_FOREVER);
        //OAL_TASK_UNSAFE_SET();

        if (NULL_PTR != stp_msg)
        {
            st_primitive_id = OAL_GET_OAL_MSG_PRIMITIVE_ID(stp_msg);
            vp_oal_msg_body = (CONST_VOID_PTR)OAL_GET_OAL_MSG_BODY(stp_msg);
            switch(st_primitive_id)
            {
                case MSG_HLS_L1_PRIMITIVE_ID:
                    l1cc_hl_msg_proc((oal_hls_msg_t*)vp_oal_msg_body);
                    break;

                default:
                    break;
            }

            /* Release the received message */
            ks_oal_msg_release(&stp_msg);
        }
    }
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID l1cc_ul_proc_main(VOID)
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/

//OAL_DTCM_DATA_SECTION UINT32 test_tx_counter[8] = {0};
OAL_DTCM_DATA_SECTION UINT32 pack_pkg_flag = 0, pingpong_flag = 1;
//OAL_DRAM_DATA_SECTION uint32 test_recv_buf_x1643[7000];
#ifdef FUN_OTD
extern UINT8 g_s16_otd_flg;
#endif
OAL_ITCM_CODE_SECTION
void trigger_cparm_pack_pkg(UINT32 u32_tx_buf_id)
{
	#if 0
    oal_msg_t* stp_msg_send = NULL_PTR;
    oal_hls_msg_t* stp_hls_msg = NULL_PTR;
    UINT32 *hls_msg_body = NULL_PTR;
	VOLATILE_UINT32_PTR src_ptr_0 = NULL_PTR, src_ptr_1 = NULL_PTR, phych_ptr = NULL_PTR;

    stp_msg_send = OAL_MSG_CREAT((L1CC_HLS_MSG_HEAD_SIZE+8)>>1);
    OAL_ASSERT(NULL_PTR != stp_msg_send, "ks_oal_msg_create failure\n");

    stp_msg_send->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
    stp_msg_send->u16_mode_type    = 7;//LTE_MODE;
    stp_msg_send->u16_standby_id   = 0;//STANDBY_0

    stp_hls_msg = (oal_hls_msg_t*)&(stp_msg_send->vp_msg_body);
    stp_hls_msg->u16_msg_size    = (L1CC_HLS_MSG_HEAD_SIZE + 8);
    stp_hls_msg->u32_msg_id      = ZZW_MACREQ_L2H_REQUEST_DATA;//P_L1_HLS_SWITCHON_IND;
    stp_hls_msg->b8_src_id       = 10;//ZZW_HLS_MAIN_TASK_ID
    stp_hls_msg->u16_pdu_size    = 0;
    stp_hls_msg->u8_phy_task_id  = 2;
    stp_hls_msg->b8_standby_id   = 0;
    stp_hls_msg->b8_mode_type    = 3;

	hls_msg_body = (UINT8 *)L1CC_GET_HLS_MSG_BODY(stp_hls_msg);
	*hls_msg_body = (&g_u8_tx_buffer[u32_tx_buf_id][0] + 0x11000000);
	
    ks_oal_dsp2arm_msg_send(stp_msg_send, NULL_PTR, 0, (UINT16)OAL_L1_HLS_WR_MB0_ID, (UINT16)OAL_SEND_IMMEDIATE, (UINT16)OAL_IPC_WAIT_FOREVER);
    #else
	UINT32 u32_buf_addr = (UINT32)(&g_u8_tx_buffer[u32_tx_buf_id][0] + 0x11000000);
	l1cc_dsp2arm_msg_send(ZZW_MACREQ_L2H_REQUEST_DATA, 4, (UINT16)OAL_L1_HLS_WR_MB0_ID, &u32_buf_addr);

	#ifdef FUN_OTD
	if(g_s16_otd_flg EQ 2)
	{
    	l1cc_dsp2arm_msg_send(60560, 0, OAL_L1_HLS_WR_MB1_ID, NULL_PTR);
    	g_s16_otd_flg = 3;
	}
	#endif
	#endif
}

OAL_ITCM_CODE_SECTION
void dsp2cparm_rssi_rpt(UINT32_PTR u32p_rssi_db)
{
    oal_msg_t *stp_msg;
    oal_hls_msg_t* stp_hls_msg;
    UINT8 *pkt_addr_ptr;
    UINT32_PTR u32p_temp = NULL_PTR;
    DOWNLINK_PHY_HEADER_TYPE *downlink_phy_ptr = NULL_PTR;
    UINT16 msg_len = sizeof(oal_hls_msg_t) - sizeof(VOID_PTR) + 8;

    pkt_addr_ptr = (UINT8 *)to_cp_hs5.get_mbuf_ptr();
    stp_msg = OAL_MSG_CREAT((msg_len + 1) >> 1);

    OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

    stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
    stp_msg->u16_mode_type    = 3;//FLIGHT_MODE;
    stp_msg->u16_standby_id   = 0;//STANDBY_0;

    stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
    stp_hls_msg->u16_msg_size   = msg_len;
    stp_hls_msg->b8_src_id      = 10;
    stp_hls_msg->u16_pdu_size   = 0;
    stp_hls_msg->u8_phy_task_id = 2;
    stp_hls_msg->u32_msg_id     = ZZW_MACREQ_L2H_PACKAGE;
    stp_hls_msg->b8_standby_id  = 0;//STANDBY_0
    stp_hls_msg->b8_mode_type   = 3;//FLIGHT_MODE;

    downlink_phy_ptr = (DOWNLINK_PHY_HEADER_TYPE *)(((UINT8_PTR)pkt_addr_ptr) + sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));
    downlink_phy_ptr->mcs = 0;
    downlink_phy_ptr->type = 13;
    downlink_phy_ptr->frame_len = 0;

    u32p_temp = (UINT32_PTR)(((UINT8_PTR)pkt_addr_ptr) + sizeof(ZZW_CHANNEL_STATUS_INDICATION_T) + sizeof(DOWNLINK_PHY_HEADER_TYPE));

    u32p_temp[0] = g_zzw_freq_val;
    ks_oal_mem_copy(&u32p_temp[1], u32p_rssi_db, 32);

    to_cp_hs5.send_mbuf(stp_msg, ARM2DSP_DL_DATA_BUFSIZE);
}

OAL_ITCM_CODE_SECTION
VOID read_arm2dsp_data(UPLINK_PHY_HEADER_TYPE *pkg_ptr, UINT32 u32_tx_buf_id)
{
    UINT8_PTR src_ptr = NULL_PTR, phych_ptr = NULL_PTR;
	UINT32 u32_head;
	UINT32 u32_i = 0;
	UINT32 u32_len = 0;
	#if 0
    if(u32_pingpong_id)
    {
        src_ptr = (UINT8_PTR)BUFFER_CPARM2DSP_UL_INDEX_1;
    } 
    else
    {
        src_ptr = (UINT8_PTR)BUFFER_CPARM2DSP_UL_INDEX_0;
    }
    #else
	src_ptr = (UINT8_PTR)&g_u8_tx_buffer[u32_tx_buf_id][0];
    #endif
	KS_LOG(0, 0xAC03, u32_tx_buf_id);
    
    u32_head = *((UINT32_PTR)src_ptr);
    if(u32_head != 0xABCDEF12)
    {
		if(u32_head == 0xA5A5A5A5)
		{
			*((UINT32_PTR)src_ptr) = 0x12345678;
			KS_LOG(0, 0xAC01, (UINT16)(0xDEAC));
		} 
		else 
		{	
			KS_LOG(0, 0xAC01, (UINT16)(0xDEAD));
		}
		ks_oal_mem_set((src_ptr + 16), 0xA5, 16);
		pkg_ptr->frame_len = 0;
		pkg_ptr->mcs = 0;
		return;		
    }

    phych_ptr = src_ptr + 8;

    #if 0
    pkg_ptr->mcs = ((UPLINK_PHY_HEADER_TYPE *)phych_ptr)->mcs;
    pkg_ptr->type = ((UPLINK_PHY_HEADER_TYPE *)phych_ptr)->type;
    pkg_ptr->frame_len = ((UPLINK_PHY_HEADER_TYPE *)phych_ptr)->frame_len;
    pkg_ptr->rcv_addr = ((UPLINK_PHY_HEADER_TYPE *)phych_ptr)->rcv_addr;
    #else
    ks_oal_mem_copy((void *)pkg_ptr,(void *)phych_ptr, sizeof(UPLINK_PHY_HEADER_TYPE));// copy data
    #endif
    //ks_oal_mem_copy(test_recv_buf_x1643,(void *)(src_ptr + 16), pkg_ptr->frame_len);// copy data
	KS_LOG(0, 0xAC01, (UINT16)((pkg_ptr->frame_len)));	

	u32_len = (pkg_ptr->frame_len << 3);
	for(u32_i = 0; u32_i < ZZW_MAX_MCS_NUM; u32_i++)
	{
		if(g_st_mcs_tbl[u32_i].u32_raw_len >= u32_len)
		{
			break;
		}
	}
	OAL_ASSERT(u32_i < ZZW_MAX_MCS_NUM, "");
	
	KS_LOG(0, 0xAC02, (u32_i<<8) | pkg_ptr->mcs);
	
	pkg_ptr->mcs = u32_i;
    *((UINT32_PTR)src_ptr) = 0x12345678;
}

/***********************************************************************************************************************
* FUNCTION
*
* VOID l1cc_ul_proc_main(VOID)
*
*
* DESCRIPTION
*
*
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   NULL
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
*
***********************************************************************************************************************/
OAL_DTCM_DATA_SECTION UINT32 u32_plcp_report_cnt = 0;
OAL_DTCM_DATA_SECTION UINT32 u32_timing_adj_test[8][2] = {0};
OAL_DTCM_DATA_SECTION UINT16 bs_send_cnt = 0, ds_send_cnt = 0;

OAL_DTCM_DATA_SECTION UPLINK_PHY_HEADER_TYPE g_ulink_pkg_head;
OAL_DTCM_DATA_SECTION UINT32 zzw_mcs_cnt = 0, dsp_mcs = 0, cparm_mcs = 0;
OAL_DTCM_DATA_SECTION UINT32 ul_cpy_time[2];


OAL_ITCM_CODE_SECTION VOID l1cc_ul_proc_common(UINT32 u32_data_type, UINT32 u32_buf_id)
{
	UINT32 tmp_index = 0;
	UINT32 u32_i = 0;
	UINT32 u32_len = 0;	
    l1cc2algo_ctrl_info_t *stp_ctrl_info;
	UPLINK_PHY_HEADER_TYPE *phy_header_ptr = (UPLINK_PHY_HEADER_TYPE *)g_zzw_sf_b;
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);	//g_st_iq_dump
	node_info_t *stp_node_info;

	if(g_u32_tx_encode_cancel)
	{
		algo_ul_tx_cancel_proc();
		g_u32_tx_encode_cancel = OAL_FALSE;
	}
	
	tmp_index = (g_u8_tx_ctrl_info_cnt_in + 1)%TX_CTRL_INFO_CNT_MAX;
	OAL_ASSERT(tmp_index != g_u8_tx_ctrl_info_cnt_out, "tx ctrl info buf FULL !!");
	
	stp_ctrl_info = &g_st_ctrl_info[g_u8_tx_ctrl_info_cnt_in];
	g_u8_tx_ctrl_info_cnt_in = tmp_index;
	
	stp_ctrl_info->u32_frame_num	= (g_s32_frame_no + 1);
	stp_ctrl_info->u8_slot_per_dec	= 1;
	stp_ctrl_info->u8_data_num		= 9; /*data total num*/
	stp_ctrl_info->u8_symb_num		= 14;
	if(stp_sarm_1643_4500->u8_assert_flg)
	{
		stp_ctrl_info->u8_magic_word	  = ZZW_PLCP_MAGIC_WORD_ASSERT;
	}
	else
	{
		stp_ctrl_info->u8_magic_word	  = ZZW_PLCP_MAGIC_WORD_NORMAL; 					
	}
	stp_ctrl_info->u8_tx_pwr	   = (UINT8)(g_zzw_tx_pwr&0xFF);
	stp_ctrl_info->u8_node_id	  = (UINT8)g_zzw_node_status.lc_id;
	stp_ctrl_info->s8_snr	  = stp_sarm_1643_4500->s8_snr;

	if(u32_data_type EQ 0)
	{
		//BS
		stp_ctrl_info->u16_mac0_len 	= phy_header_ptr->type;
		stp_ctrl_info->u16_mac1_len 	= phy_header_ptr->frame_len;
		stp_ctrl_info->u16_send_cnt 	  = bs_send_cnt++;
		
		#ifdef TX_TEST
		stp_ctrl_info->u8_data_type 	= 1;
		stp_ctrl_info->u8_mcs			= READ_REG32(0x1724fff0); 
		stp_ctrl_info->u8_node_id	  = (UINT8)1;
		#else
		stp_ctrl_info->u8_data_type 	= 0;//BS_TYPE;
		u32_len = (phy_header_ptr->frame_len << 3);
		for(u32_i = 0; u32_i < ZZW_MAX_MCS_NUM; u32_i++)
		{
			if(g_st_mcs_tbl[u32_i].u32_raw_len >= u32_len)
			{
				break;
			}
		}
		OAL_ASSERT(u32_i < ZZW_MAX_MCS_NUM, "");
		stp_ctrl_info->u8_mcs = u32_i;
		#endif
		KS_LOG(0, 0xA112, (UINT16)bs_send_cnt); 
		KS_LOG(0, 0xA114, (UINT16)phy_header_ptr->frame_len); 
		g_u8_node_id_target = 0;
		#if APC_TEST
		if(s_st_node_info_tbl.u8_is_valid)
		{
			stp_node_info = &s_st_node_info_tbl.st_node_info[0];
			stp_ctrl_info->s8_tx_pwr_expect = stp_node_info->s8_tx_pwr_expect;
			KS_LOG(0, 0xA116, (UINT16)stp_ctrl_info->s8_tx_pwr_expect); 
		}
		#endif
	}
	else
	{
		stp_ctrl_info->u8_data_type 	= 1;//DS_TYPE;
		stp_ctrl_info->u16_send_cnt 	  = ds_send_cnt++;
		#if 0
		g_ulink_pkg_head.mcs = 0;
		g_ulink_pkg_head.type = 0;
		g_ulink_pkg_head.frame_len = 0;
		g_ulink_pkg_head.rcv_addr = 0;
		#endif
		read_arm2dsp_data(&g_ulink_pkg_head, u32_buf_id);

		#if 1
		if(g_ulink_pkg_head.frame_len EQ 0)
		{
			stp_ctrl_info->u16_mac0_len 	= phy_header_ptr->type;
			stp_ctrl_info->u16_mac1_len 	= phy_header_ptr->frame_len;
			//stp_ctrl_info->u16_send_cnt 	  = bs_send_cnt++;
			
			#ifdef TX_TEST
			stp_ctrl_info->u8_data_type 	= 1;
			stp_ctrl_info->u8_mcs			= READ_REG32(0x1724fff0); 
			#else
			stp_ctrl_info->u8_data_type 	= 0;//BS_TYPE;
			u32_len = (phy_header_ptr->frame_len << 3);
			for(u32_i = 0; u32_i < ZZW_MAX_MCS_NUM; u32_i++)
			{
				if(g_st_mcs_tbl[u32_i].u32_raw_len >= u32_len)
				{
					break;
				}
			}
			OAL_ASSERT(u32_i < ZZW_MAX_MCS_NUM, "");
			stp_ctrl_info->u8_mcs = u32_i;
			#endif
			KS_LOG(0, 0xA112, (UINT16)bs_send_cnt); 
			KS_LOG(0, 0xA114, (UINT16)phy_header_ptr->frame_len); 		
		}
		else
		#endif
		{
			g_zzw_mcs = g_ulink_pkg_head.mcs;
			if(g_zzw_mcs>100)
			{
				g_zzw_mcs = g_zzw_mcs - 100;
			}

			stp_ctrl_info->u8_mcs	=  g_ulink_pkg_head.mcs;
			//stp_ctrl_info->u8_mcs   =  (ds_send_cnt + 1) % 14;
			
			stp_ctrl_info->u16_mac0_len 	= g_ulink_pkg_head.type;
			stp_ctrl_info->u16_mac1_len 	= g_ulink_pkg_head.frame_len;
		}
		g_u8_node_id_target = g_ulink_pkg_head.rcv_addr;
		#ifdef FUN_OTD
		if(g_ulink_pkg_head.node_id_target)
		{
			s_st_node_info_tbl.u32_ref_bmp |= (1 << (g_ulink_pkg_head.node_id_target - 1)); 
			stp_node_info = node_tbl_find(g_ulink_pkg_head.node_id_target);

			if(stp_node_info)
			{
				stp_ctrl_info->u8_node_id_target = g_ulink_pkg_head.node_id_target;
				KS_LOG(0, 0xA117, (UINT16)g_ulink_pkg_head.otd); 
				stp_ctrl_info->s16_otd = 0x7FFF;

				//OAL_ASSERT(stp_node_info NEQ NULL_PTR, "");
				if(((stp_node_info->s16_otd - g_ulink_pkg_head.otd > 100)\
						|| (stp_node_info->s16_otd - g_ulink_pkg_head.otd < -100)))
				{
					if(stp_node_info->s16_otd_bak != g_ulink_pkg_head.otd)
					{
						stp_node_info->s16_otd += g_ulink_pkg_head.otd;
						stp_node_info->s16_otd_bak = g_ulink_pkg_head.otd;
					}
					stp_ctrl_info->s16_otd = g_ulink_pkg_head.otd;

				}
			}
			else
			{
				stp_ctrl_info->u8_node_id_target = g_u8_node_id_target;
				stp_ctrl_info->s16_otd = 0x7FFF;
			}
		}
		else
		{
			stp_ctrl_info->u8_node_id_target = g_u8_node_id_target;
			stp_ctrl_info->s16_otd = 0x7FFF;
		}
		#else
		stp_ctrl_info->u8_node_id_target = g_u8_node_id_target;
		#endif
		stp_ctrl_info->s8_tx_pwr_expect = TX_PWR_INVALID;
		#if APC_TEST
		if(s_st_node_info_tbl.u8_is_valid)
		{
			stp_node_info = &s_st_node_info_tbl.st_node_info[0];
			stp_ctrl_info->s8_tx_pwr_expect = stp_node_info->s8_tx_pwr_expect;
			KS_LOG(0, 0xA116, (UINT16)stp_ctrl_info->s8_tx_pwr_expect); 
		}
		#else
		if(g_u8_node_id_target)
		{
			stp_node_info = node_tbl_find(g_u8_node_id_target);
			if(stp_node_info)
			{
				stp_ctrl_info->s8_tx_pwr_expect = stp_node_info->s8_tx_pwr_expect;
				stp_ctrl_info->s8_snr = stp_node_info->s8_snr;

				KS_LOG(0, 0xA116, (UINT16)stp_ctrl_info->s8_tx_pwr_expect); 		
			}
		}
		#endif
	}
	algo_data_tx_process(u32_buf_id);

}

OAL_ITCM_CODE_SECTION
VOID l1cc_ul_proc_main(VOID)
{
    oal_msg_t *stp_msg;
    oal_primitive_id_t CONST st_wait_msg_list[2] = {(UINT32)1, (UINT32)OAL_ANY_MSG_ID};
    oal_primitive_id_t st_primitive_id;
    UINT32_PTR u32p_temp = NULL_PTR;
    l1cc2algo_ctrl_info_t *stp_ctrl_info;
	UINT32 tmp_index = 0;
    
    OAL_IRQ_SAVE_AREA;

	#ifdef TX_TEST
	WRITE_REG32(0x1724fff0, 0); 
	#endif
	//*(VUINT32_PTR)(0x1724FFF0) = 11;
    while(1)
    {
        //OAL_TASK_SAFE_SET();
        /* Wait for one message forever */
        stp_msg = ks_oal_msg_receive(st_wait_msg_list, (UINT16)OAL_IPC_WAIT_FOREVER);
        //OAL_TASK_UNSAFE_SET();

        if(NULL_PTR != stp_msg)
        {
            st_primitive_id = OAL_GET_OAL_MSG_PRIMITIVE_ID(stp_msg);
            u32p_temp = OAL_GET_OAL_MSG_BODY(stp_msg);
			KS_LOG(0, 0xA110, (UINT16)st_primitive_id);	

            switch(st_primitive_id)
            {
                case MSG_UL_DS_NEXT_TX_PRIMITIVE_ID:
                {
                	#ifndef TX_TEST
                    trigger_cparm_pack_pkg(*u32p_temp);
                    #endif
                    break;
                }
                case MSG_UL_BS_PROC_PRIMITIVE_ID:
				{   
					l1cc_ul_proc_common(0, 0);
                    break;
                }
                case MSG_UL_DS_PROC_PRIMITIVE_ID:
                {
					l1cc_ul_proc_common(1, *u32p_temp);
                    break;
				}
                case MSG_DL_RSSI_REPORT_PRIMITIVE_ID:
                    dsp2cparm_rssi_rpt(u32p_temp);
                    break;
                case MSG_DL_RTC_REPORT_PRIMITIVE_ID:
                    g_zzw_node_status.lc_id = 1;
                    if(1 == g_zzw_node_status.lc_id)
                    {
                        break;
                    }
                    
                    OAL_IRQ_DISABLE;
                    OAL_ASSERT(g_zzw_rtc.u32_valid == OAL_FALSE, "set rtc confliction\n");
                    //ks_oal_mem_copy(&g_zzw_rtc, vp_msg_body, sizeof(ZZW_RTC_DSP_T));
                    g_zzw_rtc.u32_valid = OAL_TRUE;
                    OAL_IRQ_RESTORE;
                    break;

                default:
                	OAL_ASSERT(0, "WRONG MSG TYPE!");
                    break;
            }

            /* Release the received message */
            ks_oal_msg_release(&stp_msg);
            
			KS_LOG_TMRL(0xA111); 
        }
    }
}

VOID l1cc_ul_mod_main(VOID)
{
    oal_msg_t *stp_msg;
    oal_primitive_id_t CONST st_wait_msg_list[2] = {(UINT32)1, (UINT32)OAL_ANY_MSG_ID};
    oal_primitive_id_t st_primitive_id;
    
    while(1)
    {
        //OAL_TASK_SAFE_SET();
        /* Wait for one message forever */
        stp_msg = ks_oal_msg_receive(st_wait_msg_list, (UINT16)OAL_IPC_WAIT_FOREVER);
        //OAL_TASK_UNSAFE_SET();

        if(NULL_PTR != stp_msg)
        {
            st_primitive_id = OAL_GET_OAL_MSG_PRIMITIVE_ID(stp_msg);
			KS_LOG_TMRL(0xA120);

            switch(st_primitive_id)
            {
                case MSG_UL_MODULATION_PRIMITIVE_ID:
					algo_ul_data_symbproc();
                    break;

				case MSG_UL_DFT_DATA_PRIMITIVE_ID:
					algo_ul_data_ifft_pre();
					break;	

                default:
                	OAL_ASSERT(0, "WRONG MSG TYPE!");                
                    break;
            }

            /* Release the received message */
            ks_oal_msg_release(&stp_msg);
			KS_LOG_TMRL(0xA121); 
        }
    }
}

#endif

/*************************************************** END OF FILE ******************************************************/

