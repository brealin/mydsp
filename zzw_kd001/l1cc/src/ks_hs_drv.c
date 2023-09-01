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
* 1.0 2020-12-09 created by FSLI
***********************************************************************************************************************/
#undef THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID KS_HS_MSG_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "ks_hs_drv.h"
#include "zzw_common.h"
#include "ks_kd001_main.h"

extern ZZW_CHANNEL_STATUS_INDICATION_T g_zzw_ch_status;


/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
 OAL_DTCM_DATA_SECTION pkg_head_t g_pkg_head;
#if defined(CORE_X1643)
#if 0
OAL_ITCM_CODE_SECTION
UINT16 x1643_get_unread_buf_info(UINT32 pkg_max_size)
{
    UINT16 buf_num = 0;
    UINT16 loop_i = 0;
    UINT32 sum = 12;// sizeof(pkg_head_t) - sizeof(pkg_frame_t) * 512
    phych_head_t *phych_head_ptr = (phych_head_t *)HLS_L1_MB3_RD_ADDR;
    phych_frame_t *phych_frame_ptr;


    buf_num = (PHYCH_DATA_BUFNUM - phych_head_ptr->u32_read_offset/24 + phych_head_ptr->u32_write_offset/24) % PHYCH_DATA_BUFNUM;
    g_pkg_head.pkg_num = 0;
    g_pkg_head.pkg_valid_size = 0;
    if(buf_num)
    {
        g_pkg_head.pkg_max_size = pkg_max_size;
        phych_frame_ptr = (phych_frame_t *)(phych_head_ptr->u32_read_offset + HLS_L1_MB3_RD_ADDR + ((UINT32)MAILBOX_CTRL_HEAD_SIZE_IN_WORD<<1));
        for(loop_i=0;loop_i<buf_num;loop_i++)
        {
            sum = sum + phych_frame_ptr->len + sizeof(pkg_frame_t);
            if(sum>pkg_max_size)
            {
                buf_num = loop_i;
                break;
            }
            g_pkg_head.pkg_num = loop_i + 1;
            g_pkg_head.pkg_valid_size = sum;
            g_pkg_head.pkg_info[loop_i].index = phych_frame_ptr->index;
            g_pkg_head.pkg_info[loop_i].len = phych_frame_ptr->len;

			phych_frame_ptr++;
        }
    } else {
        buf_num = 0;
    }
    
    return buf_num;
}

OAL_ITCM_CODE_SECTION
void x1643_phych_read_data(pkg_head_t *pkg_ptr, UINT8_PTR dest_addr)
{
    UINT16 buf_num = pkg_ptr->pkg_num;
    UINT16 loop_i = 0;
    UINT16 copy_len = 0;
    UINT8_PTR dest_ptr = dest_addr,phych_ptr = NULL_PTR;
    phych_head_t *phych_head_ptr = (phych_head_t *)HLS_L1_MB3_RD_ADDR;
    
    copy_len = 12+ pkg_ptr->pkg_num * sizeof(pkg_frame_t);
    ks_oal_mem_copy(dest_ptr, pkg_ptr, copy_len);// copy package head info
    dest_ptr = dest_ptr + copy_len;
    for(loop_i=0;loop_i<buf_num;loop_i++)
    {
        phych_ptr = ks_oal_get_recv_buff_ptr(pkg_ptr->pkg_info[loop_i].index);
        ks_oal_mem_copy(dest_ptr, phych_ptr, pkg_ptr->pkg_info[loop_i].len);// copy data
        dest_ptr = dest_ptr + pkg_ptr->pkg_info[loop_i].len;

        phych_head_ptr->u32_read_offset = (phych_head_ptr->u32_read_offset + 24)%(PHYCH_DATA_BUFNUM*24);
        phych_head_ptr->u32_read_irq_num++;
    }
}

OAL_ITCM_CODE_SECTION
void x1643_phych_read_mbuf(pkg_head_t *pkg_ptr, UINT8_PTR dest_addr)
{
    UINT16 buf_num = pkg_ptr->pkg_num;
    UINT16 loop_i = 0;
    UINT8_PTR dest_ptr = dest_addr,phych_ptr = NULL_PTR;

    for(loop_i=0;loop_i<buf_num;loop_i++)
    {
    	OAL_ASSERT(pkg_ptr->pkg_info[loop_i].len <= (ZZW_DATA_PACKET_LEN + 4), "phych msg lenth is wrong");
        phych_ptr = ks_oal_get_recv_buff_ptr(pkg_ptr->pkg_info[loop_i].index);
        ks_oal_mem_copy(dest_ptr, phych_ptr, pkg_ptr->pkg_info[loop_i].len);// copy data
        dest_ptr = dest_ptr + pkg_ptr->pkg_info[loop_i].len;

		*((UINT32_PTR)phych_ptr) = 0x12345678;
    }
}
#endif

#else
OAL_DTCM_DATA_SECTION UINT32 test_send_count[2] = {0,0};
OAL_DTCM_DATA_SECTION UINT64 u64_dsp_recv_len = 0;
OAL_DRAM_DATA_SECTION uint32 test_recv_buf2[7000];
extern VOID _ddma_mem_copy(VOID_PTR dst_addr, VOID_PTR src_addr, UINT32 u32_len, UINT32 u32_wait);
OAL_ITCM_CODE_SECTION
VOID dl_dmac_callback()
{
	KS_LOG_TMRL(0xAC25);
}


OAL_ITCM_CODE_SECTION
VOID dl_dmad_single_copy(eKS_DMAC edmac, uint32 src_addr, uint32 dst_addr,uint32 block_ts,PDMAC_INTR_HANDLE dmad_callback_func)
{
    DMAC_NONLINK_INFO dmac_info;
	OAL_ASSERT(block_ts > 0, "");			 

    // dmac_info.ch_id = dmac_ch_request_fix(edmac, DMAC_CH0);
    dmac_info.ch_id = dmac_ch_request(edmac);
    dmac_info.src = src_addr;
    dmac_info.dst = dst_addr;
    dmac_info.block_size = block_ts;
    dmac_info.trf_type = DMAC_TRF_MEM2MEM;
    dmac_info.src_trf_mode = DMAC_TRF_SINGLE;
    dmac_info.dst_trf_mode = DMAC_TRF_SINGLE;
    dmac_info.src_width = DMAC_TR_WIDTH_128;
    dmac_info.dst_width = DMAC_TR_WIDTH_128;
    dmac_info.src_sts = DMAC_ADDR_INCREMENT;
    dmac_info.dst_sts = DMAC_ADDR_INCREMENT;
    dmac_info.src_msize	   = DMAC_MSIZE(dmac_info.block_size); // DMAC_MSIZE_1024;
    dmac_info.dst_msize	   = DMAC_MSIZE(dmac_info.block_size); // DMAC_MSIZE_1024;
    dmac_info.prior = DMAC_CH_PRIO2;

    dmac_info.intr= DMAC_CH_DMA_TRF_DONE;
    OAL_ASSERT(dmac_info.ch_id != DMAC_CH_INVALID, "dma channel req failure");
    ks_dmac_config(edmac, &dmac_info);
    ks_dmac_register_handle(edmac, dmac_info.ch_id, dmad_callback_func, NULL);
    ks_dmac_ch_start(edmac, dmac_info.ch_id);
}



OAL_ITCM_CODE_SECTION
void xc4500_phych_proc_data(UINT8_PTR src, UINT32 u32_frame_type, SINT32 s32_frame_no, SINT32 s32_rtc_value, UINT16 pkg_type, UINT16 pkg_len)
{
	oal_msg_t *stp_msg;
	oal_hls_msg_t* stp_hls_msg;
	UINT16 msg_len = sizeof(oal_hls_msg_t) - sizeof(VOID_PTR) + 8;
    // THls_Phy_buf_info mbuf_info;
    UINT16 loop_i = 0, copy_len = 0, check_pkg_len = 0;
    void *send_ptr = NULL_PTR;
    UINT8_PTR src_ptr = src;
    SINT32_PTR s32p_temp = (SINT32_PTR)src;
    UINT8_PTR u8p_temp = NULL_PTR;
    UINT16 downlink_pkg_size = 0;
    DOWNLINK_PHY_HEADER_TYPE	*downlink_phy_ptr = NULL_PTR;

	//if((src[0] == 0xA5) && (src[1] == 0xA5) && (src[2] == 0xA5) && (src[3] == 0xA5))
	if(s32p_temp[0] EQ 0xA5A5A5A5)
	{
		//test_send_count[0]++;
		return;
	}

    send_ptr = to_cp_hs5.get_mbuf_ptr();
    downlink_pkg_size = pkg_len;

    
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAD25 , (UINT16)((UINT32)send_ptr));
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAD25 , (UINT16)((UINT32)send_ptr >> 16));

	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAD25 , (UINT16)((UINT32)src_ptr));
	//KS_LOG(OAL_TRACE_LEVEL_0 , 0xAD25 , (UINT16)((UINT32)src_ptr >> 16));


	//if(downlink_pkg_size < 1000)
	if(1)
	{
    	ks_oal_mem_copy(send_ptr+sizeof(ZZW_CHANNEL_STATUS_INDICATION_T)+sizeof(DOWNLINK_PHY_HEADER_TYPE), src_ptr, downlink_pkg_size);
	}
	else
	{
		downlink_pkg_size = (((downlink_pkg_size + 15) >> 4) << 4);
		//turbo_decoder_dmad_single_copy(KS_DMAC0_CP, (UINT32)p_llr + 0x11200000, (UINT32)TURBO_DECODER_LLR_INPUT_ADDR, llr_size, test_dma);
		//turbo_decoder_dmad_single_copy(KS_DMAC0_CP, (UINT32)p_llr + 0x11200000, (UINT32)TURBO_DECODER_LLR_INPUT_ADDR, llr_size, NULL_PTR);
		//turbo_decoder_dmad_single_copy(KS_DMAC0_CP, (UINT32)p_llr, (UINT32)TURBO_DECODER_LLR_INPUT_ADDR, llr_size, test_dma);			
		//ks_oal_mem_copy_ext((VOID_PTR)TURBO_DECODER_LLR_INPUT_ADDR, (VOID_PTR)p_llr, (CONST_UINT32)llr_size, BITWIDTH_128, 1);		
		//_ddma_mem_copy((VOID_PTR)(send_ptr+sizeof(ZZW_CHANNEL_STATUS_INDICATION_T)+sizeof(DOWNLINK_PHY_HEADER_TYPE)), \
		//			(VOID_PTR)src_ptr, (CONST_UINT32)downlink_pkg_size, OAL_FALSE);
		dl_dmad_single_copy(KS_DMAC0_CP, ((UINT32)src_ptr) + 0x11200000, (UINT32)(send_ptr+sizeof(ZZW_CHANNEL_STATUS_INDICATION_T)+sizeof(DOWNLINK_PHY_HEADER_TYPE)),\
					(CONST_UINT32)downlink_pkg_size, NULL_PTR);
					
	}
	stp_msg = OAL_MSG_CREAT((msg_len + 1) >> 1);
	OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure\n");

	stp_msg->msg_primitive_id = MSG_L1_HLS_PRIMITIVE_ID;
	stp_msg->u16_mode_type	  = 3;//FLIGHT_MODE;
	stp_msg->u16_standby_id   = 0;//STANDBY_0;
	
	stp_hls_msg = (oal_hls_msg_t*)&(stp_msg->vp_msg_body);
	stp_hls_msg->u16_msg_size	= msg_len;
	stp_hls_msg->b8_src_id		= 10;
	stp_hls_msg->u16_pdu_size	= 0;
	stp_hls_msg->u8_phy_task_id = 2;
	stp_hls_msg->u32_msg_id 	= ZZW_MACREQ_L2H_PACKAGE;
	stp_hls_msg->b8_standby_id	= 0;//STANDBY_0
	stp_hls_msg->b8_mode_type	= 3;//FLIGHT_MODE;

#if 1
	u8p_temp = (UINT8_PTR)send_ptr;
	#if 0
    u8p_temp[0] = g_zzw_ch_status.node_id;
    u8p_temp[1] = g_zzw_ch_status.txpwr;    //txpwr
    u8p_temp[2] = g_zzw_ch_status.rssi;     //rssi
    u8p_temp[3] = g_zzw_ch_status.gain;     //gain
    u8p_temp[4] = g_zzw_ch_status.pathloss; //pathloss
    u8p_temp[5] = g_zzw_ch_status.snr;      //snr
    u8p_temp[6] = g_zzw_ch_status.mcs;     //mcs
    u8p_temp[7] = 0x1;
	#else
    g_zzw_ch_status.data_valid = 1;
	ks_oal_mem_copy((VOID_PTR)send_ptr, (VOID_PTR)&g_zzw_ch_status, sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));

	#endif
    downlink_phy_ptr = (DOWNLINK_PHY_HEADER_TYPE *)(((UINT8_PTR)send_ptr)+sizeof(ZZW_CHANNEL_STATUS_INDICATION_T));                     
    downlink_phy_ptr->mcs = g_zzw_ch_status.mcs;
    downlink_phy_ptr->type = pkg_type;
    downlink_phy_ptr->frame_len = pkg_len;
#endif
	
	// KS_PRINT(OAL_TRACE_LEVEL_0,"0x%x  0x%x  0x%x  0x%x\r\n",*((UINT32_PTR)(send_ptr)), *((UINT32_PTR)(send_ptr+4)),*((UINT32_PTR)(send_ptr+8)),*((UINT32_PTR)(send_ptr+12)));
    to_cp_hs5.send_mbuf(stp_msg, ARM2DSP_DL_DATA_BUFSIZE);
	//test_send_count[1]++;
}

#endif
/**************************************************** END OF FILE *****************************************************/

