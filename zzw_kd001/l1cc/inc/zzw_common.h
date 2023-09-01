#ifndef _ZZW_COMMON_H
#define _ZZW_COMMON_H

#include "ks_oal.h"
#include "ks_phych_type.h"

//#define MIMO_NEW
//#define TX_TEST
//#define BAND_WIDTH_10M
#define MIMO_2ANTS (1)
#define SCFDMA_SWITCH
#define PLCP_SOFT_COMBINE
#define MCS_AUTO_ADJ
#define MIMO_2X2
//#define FUN_OTD
//#define VECTOR_TX_TEST

#define DMA_CHAN_FOR_IQ_DUMP	 (4)
#define DMA_CHAN_FOR_IQ_DUMP_ANT2	 (5)

#ifdef DATA_QPSK	
#define ZZW_DATA_PACKET_LEN						   (ARM2DSP_DL_DATA_BUFSIZE - 4)
#else
#define ZZW_DATA_PACKET_LEN						   (ARM2DSP_DL_DATA_BUFSIZE - 4)
#endif

#define ZZW_MACREQ_L2H_BASE                        (1000)
#define ZZW_MACREQ_SET_BASE                        (2000)
#define ZZW_MACREQ_SET_DEBUG_BASE                  (2100)
#define ZZW_MACREQ_GET_BASE                        (3000)
#define ZZW_MACREQ_GET_DEBUG_BASE                  (3100)

#define ZZW_STATE_INIT                             0
#define ZZW_STATE_SCN                              1
#define ZZW_STATE_WAN                              2
#define ZZW_STATE_NET                              3
#define ZZW_STATE_UNKNOWN                          4
/*时隙配置*/
#define ZZW_SLOT_SND_SF                            4            /*发送勤务帧时隙*/
#define ZZW_SLOT_RCV_SF                            2            /*接收勤务帧时隙*/
#define ZZW_SLOT_SND_DF                            5            /*发送业务帧时隙*/
#define ZZW_SLOT_RCV_DF                            3            /*接收业务帧时隙*/
#define ZZW_SLOT_INDEX_MAX                         5            /* ZZW_SLOT_SND_DF */

#define ZZW_COMPLEX_FRAME_SIZE                     2048

#define ZZW_QUEUE_MAX_SIZE                         64
#define ZZW_QUEUE_NUM                              4
#define APC_TEST                                   (0)
#define IQ_DUMP_FRAME            (0x100 << 2)
#define IQ_DUMP_DMAC_LLI_LEN     60

#define CCA_POS_DELTA              (100)

#define SAMPLE_RATE                (30720)
#define FRAME_LEN                  (30720)
#ifdef MIMO2 
#define CP_LEN                     (256)
#define SYMBOL_LEN                 (2048)
#define GAP0_LEN                   (48)
#define AGC_LEN                    (816)
#define CCA_LEN                    (256)
#define SLOT_LEN                   (CP_LEN + SYMBOL_LEN)
#define DATA_SYM_NUM               (6)
#if 0
#define TOTAL_FRAME_SEG_NUM        (14) //framing配置块数
#define FRAME_SEG_MAP0             (0x33323100) //framing配置MAP0
#define FRAME_SEG_MAP1             (0x423332)    //framing配置MAP1
#else
#define TOTAL_FRAME_SEG_NUM        (12) //framing配置块数
#define FRAME_SEG_MAP0             (0x23332310) //framing配置MAP0
#define FRAME_SEG_MAP1             (0x2333)    //framing配置MAP1
#define GAP1_LEN                   (FRAME_LEN - GAP0_LEN - AGC_LEN - (CCA_LEN << 1) - TOTAL_SYM_NUM * SLOT_LEN - CCA_POS_DELTA)

#endif
#ifdef MIMO_2X2
#define ZZW_MAX_MCS_NUM                            (14)
//#define ZZW_MAX_MCS_NUM                            (20)
//#define ZZW_MAX_MCS_SFBC						   (14)
#else
#define ZZW_MAX_MCS_NUM                            (14)
#endif
#else
#define CP_LEN                     (144)
#define SYMBOL_LEN                 (2048)
#define GAP0_LEN                   (48)
#define GAP1_LEN                   (848)
#define AGC_LEN                    (816)
#define CCA_LEN                    (256)
#define SLOT_LEN                   (CP_LEN + SYMBOL_LEN)
#define DATA_SYM_NUM               (9)
#define TOTAL_FRAME_SEG_NUM        (16) //framing配置块数
#define FRAME_SEG_MAP0             (0x33333210) //framing配置MAP0
#define FRAME_SEG_MAP1             (0x42333332)    //framing配置MAP1
#define ZZW_MAX_MCS_NUM                            (15)

#endif
#define SC_NUM                     (1200)//子载波个数
#ifdef PLCP_SOFT_COMBINE
#define PLCP_ENCODE_LEN            (SC_NUM >> 1)
#else
#define PLCP_ENCODE_LEN            (SC_NUM)
#endif
#define AGC_CCA_LEN				   (AGC_LEN + (CCA_LEN << 1))
#define TOTAL_MOD_SYM_NUM          (DATA_SYM_NUM + 1)
#define TOTAL_SYM_NUM              (DATA_SYM_NUM + 4)

#ifdef FUN_OTD
#define TX_BUFF_HEAD_SIZE          (32)
#else
#define TX_BUFF_HEAD_SIZE          (16)
#endif
typedef enum ofdm_seg_type
{
	GAP0_TYPE = 0,
	AGC_CCA_TYPE = 1, //agc cca在同一个frame seg中
	PILOT_SYM_TYPE = 2,
	DATA_SYM_TYPE = 3,
	GAP1_TYPE = 4,
	MAX_SEG_TYPE = 5
}OFDM_SEG_TYPE;

typedef struct waveform_ofdm_cfg
{
	UINT32   u32_samp_rate;     //采样率，单位KHz, 典型配置：30720KHz
    UINT32   u32_frame_len;     //帧长，单位样点，典型配置：30720，1ms
    UINT32   u32_scs      ;     //子载波间隔，单位Hz，典型配置：15000Hz
    
	UINT16   u16_sc_num   ;     //子载波个数，典型配置：1200   
    UINT16   u16_symbol_len;    //符号长度，单位样点，典型配置：2048
    UINT16   u16_cp_len;        //cp长度，
	UINT16   u16_gp0_len;       //头gap长度
	UINT16   u16_gp1_len;       //尾gap长度
    UINT16   u16_agc_len;       //agc长度
    UINT16   u16_cca_len;       //cca长度

    UINT8    u8_data_sym_num;        //数据符号个数
    UINT8    u8_pilot_sym_num;       //导频符号个数
    UINT8    u8_total_frame_seg;     //frame seg数目, 不超过64,且必须为偶数
    UINT16   u16_frame_seg_len[MAX_SEG_TYPE];   //frame seg长度集合（长度不能超过4096），暂定不超过5种长度，对应OFDM_SEG_TYPE中5种类型
    UINT32   u32_frame_seg_map[8];   //frame seg长度索引，从0bit开始，每4bit对应一个frame seg

} WAVEFORM_OFDM_CFG;

typedef enum ZZW_MAC_MSG_TYPE
{
    /* 下行数据 */
    ZZW_MACREQ_L2H_PACKAGE = ZZW_MACREQ_L2H_BASE+1,
    /* ACK统计信息 */
    ZZW_MACREQ_L2H_ACK_STAT,
    /* 队列信息 */
    ZZW_MACREQ_L2H_Q_INFO,
    /* 异常警告 */
    ZZW_MACREQ_L2H_ERR_INFO,
    /* 初始化完成 */
    ZZW_MACREQ_INIT_DONE,
        /* ????? */
    ZZW_MACREQ_L2H_REQUEST_DATA,   //(1006)
    /* 上行数据 */
    ZZW_MACREQ_H2L_PACKAGE = ZZW_MACREQ_SET_BASE+1,
    /* 配置RTC */
    ZZW_MACREQ_SET_RTC = ZZW_MACREQ_SET_BASE + 2,
    /* 配置时隙表 */
    ZZW_MACREQ_SET_SLOT_TABLE = ZZW_MACREQ_SET_BASE + 3,
    /* 配置转发表 */
    ZZW_MACREQ_SET_FW_TABLE = ZZW_MACREQ_SET_BASE + 4,
    /* 配置勤务帧B */
    ZZW_MACREQ_SET_SF_B = ZZW_MACREQ_SET_BASE + 5,
    /* 配置MTU */
    ZZW_MACREQ_SET_MTU = ZZW_MACREQ_SET_BASE + 6,
    /* 配置节点速率等级 */
    ZZW_MACREQ_SET_RATE_MAP = ZZW_MACREQ_SET_BASE + 7,
    /* 配置节点状态 */
    ZZW_MACREQ_SET_NODE_STATE = ZZW_MACREQ_SET_BASE + 8,
    /* 配置复位 */
    ZZW_MACREQ_SET_REST = ZZW_MACREQ_SET_BASE + 9,
    /* 配置组播地址 */
    ZZW_MACREQ_SET_GRP_ADDR = ZZW_MACREQ_SET_BASE + 10,
    /* 配置跳频表 */
    ZZW_MACREQ_SET_FREQ_TABLE = ZZW_MACREQ_SET_BASE + 11,
    /* 配置工作模式 */
    ZZW_MACREQ_SET_RF_MODE = ZZW_MACREQ_SET_BASE + 12,
    /* 配置定频频率 */
    ZZW_MACREQ_SET_FREQ = ZZW_MACREQ_SET_BASE + 13,
    /* 配置发射功率 */
    ZZW_MACREQ_SET_RF_POWER = ZZW_MACREQ_SET_BASE + 14,
    /* 配置同步方式*/
    ZZW_MACREQ_SET_SYNC_MODE = ZZW_MACREQ_SET_BASE + 15,
    /* 配置明密方式 */
    ZZW_MACREQ_SET_CRYPTO_MODE = ZZW_MACREQ_SET_BASE + 16,
    /* 配置信道号 */
    ZZW_MACREQ_SET_CHANEL_NUM = ZZW_MACREQ_SET_BASE + 17,
    /* 配置信道资源个数 */
    ZZW_MACREQ_SET_CHANEL_RES_NUM = ZZW_MACREQ_SET_BASE + 18,
    /* 配置信道带宽 */
    ZZW_MACREQ_SET_CHANEL_BAND_WIDTH = ZZW_MACREQ_SET_BASE + 19,
    /* 配置工作波形 */
    ZZW_MACREQ_SET_WAVE_FORM = ZZW_MACREQ_SET_BASE + 20,
    /* 配置用户速率 */
    ZZW_MACREQ_SET_USER_SPEED = ZZW_MACREQ_SET_BASE + 21,
     /* 设置射频接收功率 */
    ZZW_MACREQ_SET_RF_RX_GAIN = ZZW_MACREQ_SET_BASE + 22,
    /* 设置射频接收频点 */
    ZZW_MACREQ_SET_RF_RX_FREQ = ZZW_MACREQ_SET_BASE + 23,
    /* 设置调制MCS */
    ZZW_MACREQ_SET_MCS = ZZW_MACREQ_SET_BASE + 24,
    
    ZZW_MACREQ_SET_CP_SHIFT = ZZW_MACREQ_SET_BASE + 26,
    /* 设置扫频点信号强度 */
    ZZW_MACREQ_SET_FREQ_SCAN,
    /* 设置扫频点信号强度 */
    ZZW_MACREQ_GET_FREQ_RSSI,
    /* 配置回环使能 */
    ZZW_MACREQ_SET_LOOPBACK_ENABLE = ZZW_MACREQ_SET_DEBUG_BASE+1,
    /* 配置模块使能 */
    ZZW_MACREQ_SET_MOD_ENABLE,
    /* 配置寄存器 */
    ZZW_MACREQ_SET_REG,
    /* 查询时隙表 */
    ZZW_MACREQ_GET_SLOT_TABLE = ZZW_MACREQ_GET_BASE+1,
    ZZW_MACCNF_GET_SLOT_TABLE,
    /* 查询转发表 */
    ZZW_MACREQ_GET_FW_TABLE,
    ZZW_MACCNF_GET_FW_TABLE,
    /* 查询勤务帧B */
    ZZW_MACREQ_GET_SF_B,
    ZZW_MACCNF_GET_SF_B,
    /* 查询MTU */
    ZZW_MACREQ_GET_MTU,
    ZZW_MACCNF_GET_MTU,
    /* 查询速率等级 */
    ZZW_MACREQ_GET_RATE_LV,
    ZZW_MACCNF_GET_RATE_LV,
    /* 查询节点状态 */
    ZZW_MACREQ_GET_NODE_STATE,
    ZZW_MACCNF_GET_NODE_STATE,
    /* 查询寄存器 */
    ZZW_MACREQ_GET_REG,
    ZZW_MACCNF_GET_REG,
    /* 查询射频工作模式 */
    ZZW_MACREQ_GET_RF_MODE,
    ZZW_MACCNF_GET_RF_MODE,
    /* 查询定频频率 */
    ZZW_MACREQ_GET_FREQ,
    ZZW_MACCNF_GET_FREQ,
    /* 查询射频功率 */
    ZZW_MACREQ_GET_RF_POWER,
    ZZW_MACCNF_GET_RF_POWER,
    /* 查询同步模式 */
    ZZW_MACREQ_GET_SYNC_MODE,
    ZZW_MACCNF_GET_SYNC_MODE,
    /* 查询明密模式 */
    ZZW_MACREQ_GET_CRYPTO_MODE,
    ZZW_MACCNF_GET_CRYPTO_MODE,
    /* 查询信道号 */
    ZZW_MACREQ_GET_CHANEL_NUM,
    ZZW_MACCNF_GET_CHANEL_NUM,
    /* 查询信道资源数量 */
    ZZW_MACREQ_GET_CHANEL_RES_NUM,
    ZZW_MACCNF_GET_CHANEL_RES_NUM,
    /* 查询信道带宽 */
    ZZW_MACREQ_GET_CHANEL_BAND_WIDTH,
    ZZW_MACCNF_GET_CHANEL_BAND_WIDTH,
    /* 查询自检状态 */
    ZZW_MACREQ_GET_SELF_CHECK,
    ZZW_MACCNF_GET_SELF_CHECK,
    /* 查询工作波形 */
    ZZW_MACREQ_GET_WAVE_FORM,
    ZZW_MACCNF_GET_WAVE_FORM,
    /* 查询用户速率 */
    ZZW_MACREQ_GET_USER_SPEED,
    ZZW_MACCNF_GET_USER_SPEED,
    /* 查询跳频表号 */
    ZZW_MACREQ_GET_FREQ_TABLE,
    ZZW_MACCNF_GET_FREQ_TABLE,
    /* 查询MAC地址 */
    ZZW_MACREQ_GET_MAC_ADDRESS,
    ZZW_MACCNF_GET_MAC_ADDRESS,
    /* 查询网络规模 */
    ZZW_MACREQ_GET_NETWORK_SCALE,
    ZZW_MACCNF_GET_NETWORK_SCALE,
    /* 查询通信跳数 */
    ZZW_MACREQ_GET_COMMUNICATION_HOPS,
    ZZW_MACCNF_GET_COMMUNICATION_HOPS,
    /* 查询网关选举 */
    ZZW_MACREQ_GET_GATEWAY_ELECTION,
    ZZW_MACCNF_GET_GATEWAY_ELECTION,
    /* 查询网络融合 */
    ZZW_MACREQ_GET_NETWORK_CONVERGENCE,
    ZZW_MACCNF_GET_NETWORK_CONVERGENCE,
    /* 查询网络号 */
    ZZW_MACREQ_GET_NETWORK_NUMBER,
    ZZW_MACCNF_GET_NETWORK_NUMBER,
    /* 查询拓扑信息 */
    ZZW_MACREQ_GET_TOP_INFO,
    ZZW_MACCNF_GET_TOP_INFO,
    /*highmac发往上层的流控帧*/
    ZZW_HIGHAMC_FLOW_CONTROL,
    /* 查询节点网络状态信息*/
    ZZW_MACREQ_GET_NODE_INFO,
    ZZW_MACCNF_GET_NODE_INFO,
    ZZW_MACREQ_GET_BLER = ZZW_MACREQ_GET_BASE + 56,
    ZZW_MACREQ_GET_CH_PARA = ZZW_MACREQ_GET_BASE + 62,
    ZZW_MACREQ_GET_MCS_TBL = ZZW_MACREQ_GET_BASE + 63,    
}ZZW_MAC_MSG_TYPES;

typedef struct _ZZW_CHANNEL_STATUS_INDICATION_T_
{
    UINT8  node_id;
    SINT8  txpwr;
    SINT8  rssi;
    UINT8  gain;

    UINT8  pathloss;
    SINT8  snr;
    UINT8  mcs;
    UINT8  data_valid;

    /*extend for mimo start*/
    SINT8  rssi_sub[2];
    SINT8  pathloss_sub[2];
    
    SINT8  snr_sub[4];
    
    UINT8  gain_sub[2];
    UINT8  txpwr_sub[2];
    #ifdef MCS_AUTO_ADJ
    SINT8  mcs_offset;
    UINT8  rsv[3];
    #endif
    /*extend for mimo end*/

}ZZW_CHANNEL_STATUS_INDICATION_T;

typedef struct uplink_phy_header
{
    UINT8    mcs;
    UINT8    type;
    UINT8    rcv_addr;
    #ifdef FUN_OTD
    UINT8    node_id_target;
    UINT32   frame_len;
    SINT16   otd;
    UINT8    rsv[14];
    #else
    UINT8    reserved;
    UINT32   frame_len;
    #endif
} UPLINK_PHY_HEADER_TYPE;

typedef struct downlink_phy_header
{
    UINT8    mcs;
    UINT8    type;
    UINT8    reserved[2];
    UINT32   frame_len;
} DOWNLINK_PHY_HEADER_TYPE;


#pragma pack(1)
typedef struct _ZZW_DATA_PACKET_T_
{
	DOWNLINK_PHY_HEADER_TYPE header;
    UINT8  data[0x32];
}ZZW_DATA_PACKET_T;

 typedef struct _ZZW_QUEUE_INFO_T_
 {
	 UINT8 u8_read_index;
	 UINT8 u8_write_index;
	 UINT8 u8_free_size;
	 THls_Phy_buf_info st_packet[ZZW_QUEUE_MAX_SIZE];
 }ZZW_QUEUE_INFO_T;

#pragma pack()

 typedef struct zzw_node_info_type
{
    UINT8  used : 2;
    UINT8  net_id : 6;
    UINT8  node_id : 6;
    UINT8  node_state : 2;
}ZZW_NODE_INFO_TYPE;

typedef struct _ZZW_RTC_T_
{
    SINT32 s32_frame_no;
    SINT32 s32_rtc_value;
}ZZW_RTC_T;

typedef struct _ZZW_RTC_DSP_T_
{
    UINT32 u32_valid;
    ZZW_RTC_T st_rtc;
}ZZW_RTC_DSP_T;

typedef struct zzw_sf_b_type
{
    ZZW_RTC_T node_rtc;
    UINT32 node_idx;
    ZZW_NODE_INFO_TYPE bs_map[32];
}ZZW_SF_B_TYPE;

typedef struct _ZZW_NODE_STATE_T_
{
    UINT8  rf_id;
    UINT8  rf_clock_lv;
    UINT8  lc_id;
    UINT8  lc_clock_lv;
    UINT8  res : 2;
    UINT8  retry_num : 3;
    UINT8  state : 3;
    UINT8  slot_num;
    UINT16 slot_len;
}ZZW_NODE_STATE_T;

typedef struct _ZZW_SLOT_TABLE_INFO_T_
{
    UINT32 u32_read_index;
    UINT32 u32_write_index;
    UINT8  u8_slot_table[2][1024];
}ZZW_SLOT_TABLE_INFO_T;

typedef struct _ZZW_QUEUE_STATUS_INFO_T_
{
    UINT8 u8_status;
    UINT8 u8_hsn;
}ZZW_QUEUE_STATUS_INFO_T;

typedef struct _ZZW_STR_CTRL_INFO_T_
{
    ZZW_QUEUE_STATUS_INFO_T st_queue[ZZW_QUEUE_NUM];
}ZZW_STR_CTRL_INFO_T;

typedef struct _ZZW_RX_INFO_T_
{
    UINT16 u16_node_state;
    UINT16 u16_slot_type;
    UINT16 u16_frame_no;
    UINT16 u16_is_continues;
}ZZW_RX_INFO_T;

typedef struct mcs_tbl_tag
{
    UINT32 u32_raw_len; //turbo input len
    UINT32 u32_len;//turbo output len
	UINT16 u16_mod_type;     //bpsk/qpsk/qam16/qam64
	UINT16 u16_rd_mode;      //3/0/1/2
	UINT16 u16_cb_num;      //1/2/2/2/3/3/6/6/6/6/6/8	
	UINT16 u16_rsv;
	UINT16 u32_raw_len_cb;	
	UINT16 u32_len_cb;	
}mcs_tbl_t;

typedef struct sram_1643_4500_tag
{
	//1643 write, 4500 read
    VUINT32   u32_rx_restart;
    
    VSINT16   s16_rx_fn;
    VUINT8    u8_rx_gain;
    VUINT8    u8_rx_state;
    VUINT8    u8_node_id_local;
    VUINT8    u8_ssap;
	VUINT8    u8_rsv_1643[6];

	//4500 write, 1643 read
    VUINT8    u8_assert_flg; //assert remote node
    VSINT8    s8_snr;        //rx snr    
    VUINT8    u8_node_id_remote; //for tx pwr loop ctrl
    VUINT8    u8_tx_pwr_remote;	 //for tx pwr loop ctrl 
	VUINT8    u32_rsv_4500[12];
}sram_1643_4500_t;
#define SRAM_1643_4500_ADDR         (0x1724FF00)
#define ZZW_PLCP_MAGIC_WORD_NORMAL  (0x94)
#define ZZW_PLCP_MAGIC_WORD_ASSERT  (0x95)

/******************************************************************************/
/*   \struct  l1cc2algo_ctrl_info_t
*    \brief   CI信息
*    \details    
*/
/******************************************************************************/
typedef struct l1cc2algo_ctrl_info_tag
{
    /*最后一次编码turbo输入bit数,为大于剩余bit数的turbo表中的最小值*/
    //UINT32  u16_last_block_bit               :16;
    UINT32  u32_frame_num;
    
    /*mac0长度 bytes*/
    UINT32  u16_mac0_len                     :12;
    /* 编码码率0:1/2,1:2/3,2:3/4:3:5/6(payload最低有效位)*/
    //UINT32  u8_code_rate                     :2;
    /* 调制阶数0:BPSK/1:QPSK/2:16QAM/3:64QAM*/
    UINT32  u8_mcs                           :4;
    /*mac1长度 bytes*/
    UINT32  u16_mac1_len                     :16;

    /*mac2长度 bytes*/
    UINT32  u16_mac2_len                     :8;
    /*几个符号做一次译码 */
    UINT32  u8_slot_per_dec                  :4;
    /*是否需要响应*/
    UINT32  u8_need_response                 :2;
    /*指示数据类型 0:rts,1:cts,2:ack,3:data*/
    UINT32  u8_data_type                     :2;
    /*发送计数器*/
    UINT32  u16_send_cnt                     :16;

    UINT8   u8_symb_num;
    /*本帧 data时隙(不包括pilot)总数*/
    UINT8   u8_data_num                      ;
    /*rsv 四字节对齐保留位,CI最多承载125bit,编码时取前120bit*/
    UINT8   u8_magic_word                    ;
    UINT8   u8_tx_pwr;
    
    UINT8   u8_node_id;    
	SINT8   s8_snr;   //4500 need tell 1643
    UINT8   u8_node_id_target; 
    SINT8   s8_tx_pwr_expect;
    
    #ifdef FUN_OTD
    SINT16  s16_otd;
    UINT8   u8_rsv[2];
	#endif
	
}l1cc2algo_ctrl_info_t;

typedef struct algo2l1cc_ctrl_info_ind_tag
{
    /*是否同步成功标志位,1:成功,0:失败 */
    UINT32                              u32_sync_result;

     /*控制信息译码结果,1:成功,0:失败*/
    UINT32                              u32_crc;
   
    /*控制信息内容*/
    l1cc2algo_ctrl_info_t               st_ctrl_info; 

    /*snr,0xFFFF表示无效值 */
    SINT32                              s32_snr;

    /*rssi,0xFFFF表示无效值 */
    SINT32                              s32_rssi;    
    
    /*相对于接收到的第一个IQ数据偏移位置,单位samples*/
    UINT64                              u64_sync_position; 
}algo2l1cc_ctrl_info_ind_t;

#define ZZW_PLCP_BIT_LEN    ((sizeof(l1cc2algo_ctrl_info_t) << 3) + 24)
//#define ZZW_PLCP_BIT_LEN    (224 + 24)

#define MAX_NODE_NUM        (32)

typedef struct node_info_tag
{
	UINT8  u8_node_id;
	SINT8  s8_tx_pwr;
	SINT8  s8_snr;
	UINT8  u8_gain;
	
	UINT16 u16_send_cnt;
	SINT8  s8_tx_pwr_expect;
	UINT8  u8_rsv;
	
	SINT32 s32_afc_angle;
	SINT32 s32_cca_pos;	
	UINT32 u32_rx_fn;
	
	UINT16  u16_dec_cnt;//dec cnt
	UINT16  u16_suc_cnt;//dec success cnt
	
	SINT8  s8_mcs_offset;
	UINT8  u8_mcs;
	SINT8  s8_mcs_direction; //0 mcs down, 1 mcs up
	UINT8  u8_mcs_punish;
	#ifdef FUN_OTD
	SINT16 s16_otd;
	SINT16 s16_otd_bak;

	UINT16 u16_otd_rpt_fn;
	UINT16 u16_rsv;
	#endif

}node_info_t;
                                
typedef struct node_info_tbl_tag
{
 	UINT8 u8_is_valid;
	UINT8 u8_node_num;
	#ifdef FUN_OTD
	UINT8 u8_otd_adj_flg;
	UINT8 u8_ref_nodeid;//本地定时参考节点号

	SINT16 s16_otd;
	SINT16 s16_otd_bak;
	
	UINT32 u32_ref_bmp;//定时参考关系节点bitmap，对应bit为1时，代表该节点定时参考本节点，暂时只支持32节点
	#else
    UINT8   u8_rsv[2];
	#endif
	node_info_t st_node_info[MAX_NODE_NUM];
}node_info_tbl_t;

#pragma pack(1)
typedef struct mcs_des_tag
{
	UINT8 u8_mcs_lvl;
	SINT8 s8_lower_limit;
	SINT8 s8_upper_limit;
	UINT16 u16_payload_len; //UNIT: BYTE
} mcs_des_t;
#pragma pack()

#pragma pack(1)
typedef struct mcs_tbl_des_tag
{
	UINT8 u8_mcs_num;
	mcs_des_t st_mcs_des[25];
} mcs_tbl_des_t;
#pragma pack()

extern mcs_tbl_t g_st_mcs_tbl[ZZW_MAX_MCS_NUM];
extern WAVEFORM_OFDM_CFG g_st_waveform_cfg;

#if (defined CP1_X1643) || (defined CP1_XC4500)
#define RXDFE_DMAC_LLI_ADDR (0x17032800)
#define RX_DEBUG_INFO_ADDR  (0x1702F000)
#else
#define RXDFE_DMAC_LLI_ADDR (0x17042800)
#define RX_DEBUG_INFO_ADDR  (0x17058000)
#endif

#endif
