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
* 1.0 2020-02-14 created by can.yin
***********************************************************************************************************************/

#undef THIS_FILE_NAME_ID
#define THIS_FILE_NAME_ID   RF_DRV_ZZW_FILE_ID

/***********************************************************************************************************************
* INCLUDE FILES
***********************************************************************************************************************/
#include "ks_oal.h"
#include "ks_rf_rffe.h"
#include "ks_rf_spi.h"
#include "ks_rf_ctrl.h"

#include "ks_framing.h"
#include "ks_rxdfe.h"
#include "rxdfe_drv.h"

#include "ks_typedef.h"
#include "ks_txdfe.h"
#include "ofdm_rf_drv_proc.h"
#include "ofdm_txdfe_proc.h"
#include "ofdm_rxdfe_proc.h"
#include "zzw_common.h"
#include "ks_kd001_main.h"
#include "rxdfe_intf_reg.h"
#include "ks_phych_func.h"

#include "rf_spi.h"

/***********************************************************************************************************************
* TYPE DEFINITION
***********************************************************************************************************************/
#define RFOP_2_TYPE(rf_op) ((rf_op) >> 1)
#define FREQ_THRES          (3000000)
#define FDD_DELTA_FREQ      (50000)

//gpio0_on, 0x1C; gpio0_off, 0x1D; gpio18_on, 0x82;gpio18_off, 0x83;
#define GPIO_ON_EVENT_BELOW18(gpio_num)   (0x1C + (gpio_num << 1))
#define GPIO_OFF_EVENT_BELOW18(gpio_num)  (0x1D + (gpio_num << 1))

#define GPIO_ON_EVENT_AFTER18(gpio_num)   (0x82 + (gpio_num << 1))
#define GPIO_OFF_EVENT_AFTER18(gpio_num)  (0x83 + (gpio_num << 1))

#ifdef DVB
#define RF_SPI_USED  (DD_SPI0)
#else
#define RF_SPI_USED  (DD_SPI0)
#endif
/***********************************************************************************************************************
* GLOBAL VARIABLES DEFINITION
***********************************************************************************************************************/
DTCM_RF_DRV_DATA_SECTION SINT32 g_s32_frame_no = 0;
DTCM_RF_DRV_DATA_SECTION UINT32 g_u32_current_op = NOP;
DTCM_RF_DRV_DATA_SECTION VOLATILE_UINT32 g_u32_next_op = NOP;
DTCM_RF_DRV_DATA_SECTION UINT32 g_rxdfe_timing_ext_indication = OAL_FALSE;
DTCM_RF_DRV_DATA_SECTION DFE_TIMING_INFO_T g_dfe_timing_info = {0,0,0,0,0};

OAL_DTCM_DATA_SECTION UINT32 g_u32_1st_frame = 1;
OAL_DTCM_DATA_SECTION UINT32 g_u32_1st_rtc_adj = 1;
OAL_DTCM_DATA_SECTION UINT32 g_u32_tx_encode_cancel = OAL_FALSE;
OAL_DTCM_DATA_SECTION UINT8  g_u8_evt_int_cnt = 0;
#ifdef FUN_OTD
OAL_DTCM_DATA_SECTION UINT8 g_s16_otd_flg = 0;
#endif
//RX_INFO_DATA_SECTION VOLATILE_UINT32 g_u32_rx_restart = 0; 

#define CX9261_FREQ_SET_SPI_NUM  (10)
OAL_DTCM_DATA_SECTION UINT16 g_u16_freq_reg[CX9261_FREQ_SET_SPI_NUM][4] = 
								 {
	 								 {0xF3, 0xC0, 0x3A, 0x001}, // N
	 			                     {0xF3, 0xC1, 0x30, 0x001}, // 1
	 	                             {0xF3, 0xC2, 0x65, 0x001}, // <0>
	 	                             {0xF3, 0xC3, 0x00, 0x001}, // <8:1>
	 	                             {0xF3, 0xC4, 0x00, 0x001}, // <16:9>
	 	                             {0xF3, 0xC5, 0x00, 0x001}, // <24:17>
	 	                             {0xF0, 0x11, 0x80, 0x001},
	 	                             //{0xF0, 0x0E, 0x80, 0x001}, //RX IQ CAL
	 								 {0xF0, 0x0E, 0x80, 0x226}, //RX IQ CAL delay 14us
	 	                             {0xF0, 0x11, 0x00, 0x001},
	 	                             {0xF0, 0x0E, 0x00, 0x001} //RX IQ CAL
                                 };// RX IQ CAL
RXDFE_DRV_DTCM_DATA_SECTION
UINT8 cx9261_vco_map[12][2] = {
    //vco mode //vco cfg
    {1, 0}, //0, CX9261_LO_FREQ_RANGE_2700MHZ_TO_1900MHz
    {0, 0}, //1, CX9261_LO_FREQ_RANGE_1900MHZ_TO_1350MHz
    {1, 1}, //2, CX9261_LO_FREQ_RANGE_1350MHZ_TO_950MHz
    {0, 1}, //3, CX9261_LO_FREQ_RANGE_950MHZ_TO_675MHz
    {1, 2}, //4, CX9261_LO_FREQ_RANGE_675MHZ_TO_475MHz
    {0, 2}, //5, CX9261_LO_FREQ_RANGE_475MHZ_TO_337p5MHz
    {1, 3}, //6, CX9261_LO_FREQ_RANGE_337p5MHZ_TO_237p5MHz
    {0, 3}, //7, CX9261_LO_FREQ_RANGE_237p5MHZ_TO_168p75MHz
    {1, 4}, //8, CX9261_LO_FREQ_RANGE_168p75MHZ_TO_118p75MHz
    {0, 4}, //9, CX9261_LO_FREQ_RANGE_118p75MHZ_TO_84p375MHz
    {1, 5}, //10,CX9261_LO_FREQ_RANGE_84p375MHZ_TO_59p375MHz
    {0, 5}  //11,CX9261_LO_FREQ_RANGE_59p375MHZ_TO_42p1875MHz
};
DTCM_RF_DRV_DATA_SECTION VOLATILE_UINT32 g_u32_next2_op = NOP;
DTCM_RF_DRV_DATA_SECTION VOLATILE_UINT32 g_u32_next3_op = NOP;
DTCM_RF_DRV_DATA_SECTION UINT32 s_u32_frm_int_cnt = 0;

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_nop[] = {
    0x80000010, /*delay event*/
    0x800002B0, /*delay event*/

    0x80000020, /*0x2E0:delay event*/
    0x80000008, /*0x2E8:delay event*/
    0x80000008, /*0x2F0:delay event*/

    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000013C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_rx2nop[] = {
    0x80000010, /*delay event*/
    0x800001D8, /*delay event*/

    /*Rx off*/
    0x91410010, /*0x2D0:close rx_rffe in advance 0x28*/
    0x80000018 | (GPIO_OFF_EVENT_BELOW18(12) << 16),//0x80350018, /*0x2D8:set gpio#12 low, TRX_SW2 OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(13) << 16),//0x80370008, /*0x2E8:set gpio#13 low, LNA OFF*/
    0x80000010, /*0x2F0:delay event*/

    0x80000454, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000023C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_tx2nop[] = {
    0x80000010, /*delay event*/
    0x800001B0, /*delay event*/

	#if 0//def MIMO2
	0x80000008,
    0x80000010, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	0x80000008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
    /*Tx off*/
    0x91430010, /*0x2D0:close tx_rffe in advance 0x28*/
    0x80000018 | (GPIO_OFF_EVENT_BELOW18(15) << 16),//0x803B0018, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(14) << 16),//0x80390008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	#ifdef DVB
	0x80000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0x80250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0x80000008 | (GPIO_OFF_EVENT_BELOW18(11) << 16),//0x80330008, /*0x2E8:set gpio#11 low, RF_SW1 switch to RX*/
	#endif
	#endif


    0x80000008, /*0x2F0:delay event*/

    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000023C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_tx2rx[] = {
    0x80000010, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0x80000128, /*delay event*/
	#if 0//def MIMO2
	0x80000008,
    0x80000010, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	0x80000008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	0x80000008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	0x80000008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
    /*Tx off & Rx on*/
    0x91430010, /*0x2C8-0x70:close tx_rffe in advance 0x30*/

    0x80000018 | (GPIO_OFF_EVENT_BELOW18(11) << 16),//0x80330018, /*0x2D0-0x70:set gpio#11 low, RF_SW1 switch to OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(15) << 16),//0x803B0008, /*0x2D8-0x70:set gpio#15 low, PA OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(14) << 16),//0x80390008, /*0x2E0-0x70:set gpio#14 low, PA LNA OFF*/
    0x80000008 | (GPIO_ON_EVENT_BELOW18(13) << 16),//0x80360008, /*0x2E8-0x70:set gpio#13 high, LNA ON*/

	#ifdef DVB
	0x80000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0x80250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0x80000008 | (GPIO_ON_EVENT_BELOW18(12) << 16),//0x80340008, /*0x2F0-0x70:set gpio#12 high, RF_SW2 switch to RX*/
	#endif
	#endif
    0x91400008, /*0x2F8-0x70:open rx_rffe in advance 0x70*/

    0x800001CC, /*delay event, 570 samples, GP#0 END*/
    0xA0000100, /*delay event, 384 samples, AGC*/
    0x80000378, /*delay event, 188 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000055C, /*delay event*/
    0x80000008, /*delay event, rxdfe_sync*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_nop2rx[] = {
    0x80000010, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0x80000130, /*delay event*/

    /*Rx on*/
    0x80000020 | (GPIO_ON_EVENT_BELOW18(13) << 16),//0x80360020, /*0x2E0-0x70:set gpio#13 high, LNA ON*/
	#ifdef DVB
	0x80000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0x80250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0x80000008 | (GPIO_ON_EVENT_BELOW18(12) << 16),//0x80340008, /*0x2F0-0x70:set gpio#12 high, RF_SW2 switch to RX*/
	#endif
    0x91400010, /*0x2F8-0x70:open rx_rffe in advance 0x70*/

    0x800001CC, /*delay event, 570 samples, GP#0 END*/
    0xA0000100, /*delay event, 384 samples, AGC*/
    0x80000378, /*delay event, 188 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000055C, /*delay event*/
    0x80000008, /*delay event, rxdfe_sync*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_rx2rx[] = {
    0x80000010, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0x800001A0, /*delay event*/

    0x80000020, /*0x2E0:delay event*/
    0x80000008, /*0x2E8:delay event*/
    0x80000008, /*0x2F0:delay event*/

    0x8000015C, /*delay event, 570 samples, GP#0 END*/
    0xA0000100, /*delay event, 384 samples, AGC*/
    0x80000378, /*delay event, 188 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000055C, /*delay event*/
    0x80000008, /*delay event, rxdfe_sync*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};

DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_rx2tx[] = {
    0x80000010, /*delay event*/
    #if MIMO_2ANTS
    0x91620030, /*0x40:start frame(dma)*/
    0x91630030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
    0x91450180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
    #else
    0x91620030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
	#endif
    0x800001A8, /*0x2C0:delay event*/
    0x80000008, /*0x2C0:delay event*/
    /*Rx off & Tx on*/
    0x91410008, /*0x2C8:close rx_rffe in advence 0x30*/
    0x80000010 | (GPIO_OFF_EVENT_BELOW18(12) << 16),//0x80350010, /*0x2D0:set gpio#12 low, RF_SW2 switch to OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(13) << 16),//0x80370008, /*0x2D8:set gpio#13 low, LNA OFF*/
    0x80000008 | (GPIO_ON_EVENT_BELOW18(15) << 16),//0x803A0008, /*0x2E0:set gpio#15 high, PA ON*/
    0x80000008 | (GPIO_ON_EVENT_BELOW18(14) << 16),//0x80380008, /*0x2E8:set gpio#14 high, PA LNA ON*/
	#ifdef DVB
	0x80000008 | (GPIO_ON_EVENT_BELOW18(4) << 16),//0x80240008, /*0x3F8:set gpio#4 high, RF_SW2 switch to TX*/
	#else
	0x80000008 | (GPIO_ON_EVENT_BELOW18(11) << 16),//0x80320008, /*0x2F0-0x70:set gpio#11 high, RF_SW2 switch to RX*/
	#endif
    0x91420008, /*0x2F8:open tx_rffe*/

    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/

    #ifdef MIMO2
    0x80004C00, /*delay event*/

    /*GP1*/
    0x91430008, /*0x2C8-0x70:close tx_rffe in advance 0x30*/
    0x80000018 | (GPIO_OFF_EVENT_BELOW18(15) << 16),//0x803B0018, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(14) << 16),//0x80390008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	#ifdef DVB
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0xA0250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(11) << 16),//0xA0330008, /*0x2E8:set gpio#11 low, RF_SW1 switch to RX*/
	#endif    
    0x80002E1C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000008  /*last event, remained 0x20 cycle*/
    #else
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000023C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/


    #endif
};


DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_tx2tx[] = 
#ifdef MIMO2
{
    0x80000010, /*delay event*/
    #if MIMO_2ANTS
    0x91620030, /*0x40:start frame(dma)*/
    0x91630030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert~{#,A,Px7"KMV!D#J=OB#,=v5ZR;8v~}Tx~{V!PhR*EdVC~}*/
    0x91450180, /*0x190:start cp_insert~{#,A,Px7"KMV!D#J=OB#,=v5ZR;8v~}Tx~{V!PhR*EdVC~}*/
	#else
    0x91620030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert~{#,A,Px7"KMV!D#J=OB#,=v5ZR;8v~}Tx~{V!PhR*EdVC~}*/
	#endif
    0x800001A8, /*0x2C0:delay event*/
    0x80000008, /*0x2C0:delay event*/

    0x80000020 | (GPIO_ON_EVENT_BELOW18(15) << 16),//0x803A0020, /*0x2E0:set gpio#15 high, PA ON*/
    0x80000008 | (GPIO_ON_EVENT_BELOW18(14) << 16),//0x80380008, /*0x2E8:set gpio#14 high, PA LNA ON*/
	#ifdef DVB
	0x80000008 | (GPIO_ON_EVENT_BELOW18(4) << 16),//0x80240008, /*0x3F8:set gpio#4 high, RF_SW2 switch to TX*/
	#else
	0x80000008 | (GPIO_ON_EVENT_BELOW18(11) << 16),//0x80320008, /*0x2F0-0x70:set gpio#12 high, RF_SW2 switch to RX*/
	#endif
    /*Tx on*/
    0x91420008, /*0x2F8:open tx_rffe*/
    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80004C00, /*delay event*/

    /*GP1*/
    0x91430008, /*0x2C8-0x70:close tx_rffe in advance 0x30*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(15) << 16),//0x803B0018, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(14) << 16),//0x80390008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	#ifdef DVB
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0xA0250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(11) << 16),//0xA0330008, /*0x2E8:set gpio#11 low, RF_SW1 switch to RX*/
	#endif    
    0x80002E1C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000008  /*last event, remained 0x20 cycle*/
};
#else
{
    0x80000010, /*delay event*/
    #if MIMO_2ANTS
    0x91620030, /*0x40:start frame(dma)*/
    0x91630030, /*0x40:start frame(dma)*/
	#else
    0x91620030, /*0x40:start frame(dma)*/
	#endif
    0x800001B0, /*0x2C0:delay event*/

    0x80000020, /*0x2E0:delay event*/
    0x80000008, /*0x2E8:delay event*/
    0x80000008, /*0x2F0:delay event*/
    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000023C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
};
#endif
DTCM_RF_DRV_DATA_SECTION
STATIC UINT32 s_u32_event_table_nop2tx[] = {
    0x80000010, /*delay event*/
    #if MIMO_2ANTS
    0x91620030, /*0x40:start frame(dma)*/
    0x91630030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
    0x91450180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
	#else
    0x91620030, /*0x40:start frame(dma)*/
    0x91440180, /*0x190:start cp_insert，连续发送帧模式下，仅第一个Tx帧需要配置*/
	#endif
    0x800001A8, /*0x2C0:delay event*/
    0x80000008, /*0x2C0:delay event*/

    0x80000020 | (GPIO_ON_EVENT_BELOW18(15) << 16),//0x803A0020, /*0x2E0:set gpio#15 high, PA ON*/
    0x80000008 | (GPIO_ON_EVENT_BELOW18(14) << 16),//0x80380008, /*0x2E8:set gpio#14 high, PA LNA ON*/
	#ifdef DVB
	0x80000008 | (GPIO_ON_EVENT_BELOW18(4) << 16),//0x80240008, /*0x3F8:set gpio#4 high, RF_SW2 switch to TX*/
	#else
	0x80000008 | (GPIO_ON_EVENT_BELOW18(11) << 16),//0x80320008, /*0x2F0-0x70:set gpio#12 high, RF_SW2 switch to RX*/
	#endif
    /*Tx on*/
    0x91420008, /*0x2F8:open tx_rffe*/

    0x8000047C, /*delay event, 570 samples, GP#0 END*/
    0x80000478, /*delay event, 572 samples, AGC END*/
    0x80000400, /*delay event, 512 samples, CCA END*/

    /*Traffic Data(13 data slot)*/
    0x80006698, /*delay event*/
    0x80000008, /*delay event*/
	#ifdef MIMO2
    0x80004C00, /*delay event*/

    /*GP1*/
    0x91430008, /*0x2C8-0x70:close tx_rffe in advance 0x30*/
    0x80000018 | (GPIO_OFF_EVENT_BELOW18(15) << 16),//0x803B0018, /*0x2D8:set gpio#15 low, PA OFF*/
    0x80000008 | (GPIO_OFF_EVENT_BELOW18(14) << 16),//0x80390008, /*0x2E0:set gpio#14 low, PA LNA OFF*/
	#ifdef DVB
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(4) << 16),//0xA0250008, /*0x3F8:set gpio#4 low, RF_SW2 switch to RX*/
	#else
	0xA0000008 | (GPIO_OFF_EVENT_BELOW18(11) << 16),//0xA0330008, /*0x2E8:set gpio#11 low, RF_SW1 switch to RX*/
	#endif
    0x80002A1C, /*delay event*/

    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000008  /*last event, remained 0x20 cycle*/
    #else
    0x80007800, /*delay event*/

    /*GP1*/
    0x8000023C, /*delay event*/
    0x80000008, /*delay event*/
    0x80000008, /*delay event*/
    0xC0000010  /*last event, remained 0x20 cycle*/
    #endif
};

DTCM_RF_DRV_DATA_SECTION
RF_SCH_FP rf_sch_fsm[3][3] =
{   /* next_op = nop, next_op = rx, next_op = tx, */
    {rf_sch_nop2nop, rf_sch_nop2rx, rf_sch_nop2tx}, /*current op = nop*/
    {rf_sch_rx2nop,  rf_sch_rx2rx,  rf_sch_rx2tx }, /*current op = rx*/
    {rf_sch_tx2nop,  rf_sch_tx2rx,  rf_sch_tx2tx }  /*current op = tx*/
};

RXDFE_DRV_DTCM_DATA_SECTION STATIC UINT32 s_u32_rxdfe_sync_event_addr = 0;
RXDFE_DRV_DTCM_DATA_SECTION VOLATILE_UINT32 g_u32_spi_wait_cnt = 0;
RXDFE_DRV_DTCM_DATA_SECTION UINT32 g_u32_scan_gain_level = 0;

RXDFE_DRV_DTCM_DATA_SECTION UINT8 g_u8_gain_set_flg = 0;
RXDFE_DRV_DTCM_DATA_SECTION UINT8 g_u8_rx_gain_idx = 0;

RXDFE_DRV_DTCM_DATA_SECTION STATIC UINT16 s_u16_gain_reg[8][4] = {{0x0F, 0x00, 0x3F, 0x60},  /*01db*/
                                          {0x1F, 0x0C, 0x3F, 0x60},  /*10db*/
                                          {0x3F, 0x1C, 0x3F, 0x60},  /*20db*/
                                          {0x3F, 0x2D, 0x3F, 0x60},  /*30db*/
                                          {0x3F, 0x27, 0x3F, 0x60},  /*40db*/
                                          {0x3F, 0x2F, 0x7F, 0x62},  /*50db*/
                                          {0x3F, 0x2F, 0xFF, 0x60},  /*60db*/
                                          {0x3F, 0x2F, 0xFF, 0x65}}; /*65db*/


RXDFE_DRV_DTCM_DATA_SECTION
UINT32 log2_table[64] =
{
    0xFC0FC0FC, 0x0016E797, 0xF4898D60, 0x0043ACE2,
    0xED7303B6, 0x006F2109, 0xE6C2B448, 0x0099574F,
    0xE070381C, 0x00C2615F, 0xDA740DA7, 0x00EA4F72,
    0xD4C77B03, 0x0111307E, 0xCF6474A9, 0x0137124D,
    0xCA4587E7, 0x015C01A4, 0xC565C87b, 0x01800A56,
    0xc0c0c0c1, 0x01a33761, 0xbc52640c, 0x01c592fb,
    0xb81702e0, 0x01e726aa, 0xb40b40b4, 0x0207fb51,
    0xb02c0b03, 0x0228193f, 0xac769184, 0x0247883b,
    0xa8e83f57, 0x02664f8d, 0xa57eb503, 0x02847610,
    0xa237c32b, 0x02a20231, 0x9f1165e7, 0x02bef9ff,
    0x9c09c09c, 0x02db632d, 0x991f1a51, 0x02f7431f,
    0x964fda6c, 0x03129ee9, 0x939a85c4, 0x032d7b5a,
    0x90fdbc09, 0x0347dcfe, 0x8e78356d, 0x0361c825,
    0x8c08c08c, 0x037b40e4, 0x89ae408a, 0x03944b1c,
    0x8767ab5f, 0x03acea7c, 0x85340853, 0x03c52286,
    0x83126E98, 0x03dcf68e, 0x81020408, 0x03f469c2,
};

OAL_DTCM_DATA_SECTION UINT16 g_u32_tx_buf_out = 0;
OAL_DTCM_DATA_SECTION UINT16 g_u32_tx_buf_in = 0;
OAL_DTCM_DATA_SECTION UINT16 g_u32_tx_enc_cnt = 0;
OAL_DTCM_DATA_SECTION UINT16 g_u32_tx_evt_cnt = 0;
#ifdef KD001_RF8242
OAL_DTCM_DATA_SECTION UINT16 g_u16_rf8242_ini_flg = 0;
#endif

//DTCM_RF_DRV_DATA_SECTION STATIC UINT16 s_u16_node_state = SCAN_STATUS;

/***********************************************************************************************************************
* EXTERN VARIABLES DECLARATION
***********************************************************************************************************************/
extern UINT8 g_zzw_slot_table[1024];
extern UINT16 rf_cx9261_cfg[95][4];
extern UINT16 rf_cx9261_cfg_reload[][4];

extern ZZW_RTC_DSP_T g_zzw_rtc;
extern ZZW_NODE_STATE_T g_zzw_node_status;
extern SINT16 g_zzw_tx_pwr;
extern UINT16 g_zzw_rx_gain;
extern UINT32 g_zzw_freq_val;
extern UINT32 g_zzw_freq_val_dl;
extern UINT32 g_zzw_freq_set_flag;
extern UINT16 g_zzw_freq_scan_flag;
extern UINT16 g_zzw_freq_scan_star;
extern UINT32 g_u32_build_time;
extern UINT32 g_u32_build_date;
extern SINT32 g_s32_state_change;
extern UINT16 g_zzw_freq_scan_end;
extern UINT32 g_u32_rssi_cnt;

/***********************************************************************************************************************
* FUNCTION rf_gpio_ctrl_ad9361_rst
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
RF_DRV_ITCM_CODE_SECTION
SINT32 ks_logx(UINT32 u32_base, UINT32 u32_value)
{
    UINT32 r = 1, d, x, index;
    SINT32 n0, r0, q0, q1;

    if(u32_value < 0x80000000)
    {
        r = exp_acW_acZ(u32_value) + 2;
    }

    d = u32_value<<r;

    n0 = (32 - r)<<26; /*get integer*/

    index = d>>27;

    x = log2_table[index*2];      /*1/a at Q32*/
    q0 = log2_table[index*2 + 1]; /*lb(a) at Q26*/

    x += (d>>16)*(x>>16);

    /*convert to signed*/
    r0 = (SINT32)x; /*x*/
    n0 += q0;

    q0 = (r0>>16)*(0x5555);           /*x/3*/
    q1 = (r0>>16)*(q0>>16) - (r0>>1); /*x^2/3 - x/2*/
    r0 += (q1>>16)*(r0>>16);          /*x-x^2/2+x^3/3*/
    n0 += (0x05c5)*(r0>>16);

    if(10 == u32_base)
    {
        /*(x-x^2/2+x^3/3)/ln2*/
        n0 = (n0>>16)*(0x4D10); /*2^32/log2(10)*/
    }

    return n0;
}
DTCM_RF_DRV_DATA_SECTION
UINT32 u32_rf_read_sav[4] = {0};

RF_DRV_ITCM_CODE_SECTION
VOID rf_drv_read_regs()
{
	rf_spi_AHB_control_dsp();

	rf_spi_read_reg(RF_SPI_USED, 0x6D, &u32_rf_read_sav[0], KS_RF_CXPS9261);
	rf_spi_en();

	rf_spi_read_reg(RF_SPI_USED, 0x6C, &u32_rf_read_sav[1], KS_RF_CXPS9261);
	rf_spi_en();

	rf_spi_read_reg(RF_SPI_USED, 0x5B, &u32_rf_read_sav[2], KS_RF_CXPS9261);
	rf_spi_en();

	rf_spi_read_reg(RF_SPI_USED, 0x5C, &u32_rf_read_sav[3], KS_RF_CXPS9261);
	rf_spi_en();

    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC201 , (UINT16)u32_rf_read_sav[0]);    
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC202 , (UINT16)u32_rf_read_sav[1]);	
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC203 , (UINT16)u32_rf_read_sav[2]);
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC204 , (UINT16)u32_rf_read_sav[3]);

}

RF_DRV_ITCM_CODE_SECTION
VOID rf_drv_deinit()
{
	rf_spi_AHB_control_dsp();
	rf_spi_config_power_off(RF_SPI_USED, 11, KS_RF_CXPS9261, rf_cx9261_cfg_reload);
}
#ifdef KD001_RF8242
extern UINT16 rf_cx8242_cfg[][4];
RF_DRV_ITCM_CODE_SECTION
void rf_drv_init8242()
{
    UINT32 u32_tmr_start = 0;
    UINT32 u32_tmr_end = 0;
	UINT32 u32_reg_val = 0;
    UINT32 i;
    OAL_IRQ_SAVE_AREA;

	u32_reg_val = READ_REG32(0x81F020);
	if((u32_reg_val EQ OAL_MEM_SPECIAL_TAG) && (g_u16_rf8242_ini_flg EQ 0))
	{
		OAL_IRQ_DISABLE_ALL;
		g_u16_rf8242_ini_flg = 1;
		#if 0
		if((u32_reg_val EQ OAL_MEM_SPECIAL_TAG))
		{
		    ks_oal_mem_set(g_zzw_slot_table, 0x42, sizeof(g_zzw_slot_table));
			WRITE_REG32(0x1724FFF8, 1); 
		}
		else
		{
			ks_oal_mem_set(g_zzw_slot_table, 0x22, sizeof(g_zzw_slot_table));
			WRITE_REG32(0x1724FFF8, 2); 
		}
		#else
		//ks_oal_mem_set(g_zzw_slot_table, 0x44, sizeof(g_zzw_slot_table));
		//WRITE_REG32(0x1724FFF8, 1); 
		#endif
		ks_oal_delay(10000 * 4 * 4000);// 4s
		
		rf_spi_config(RF_SPI_USED, 434, KS_RF_CXPS8242, rf_cx8242_cfg); 
		//WRITE_REG32(0x1724FFF8, 2); 

		


		#if 0
		rf_tool_ad9361_init();
		ks_oal_delay(10000 * 4 * 4000);// 4s
		rf_ctrl_enable();
		rf_ctrl_interr_mask_config(g_u32_interr_mask);
		#endif
		algo_tx_proc_init();
		KS_LOG_TMRL(0xD401);
		framing_config_init(0);
		#if MIMO_2ANTS
		framing_config_init(1);
		#endif
		KS_LOG_TMRL(0xD402);
		rxdfe_init(0);
		KS_LOG_TMRL(0xD403);
		txdfe_init(0);
		KS_LOG_TMRL(0xD404);
		//turbo_encoder_init();
		// turbo_encoder_set_callback_func(scfde_tx_turbo_enc_callback);
		rf_drv_init();
		KS_LOG_TMRL(0xD405);
		
		ks_oal_delay(10000*4*4000); 
		rf_ctrl_enable();

		
		KS_LOG_TMRL(0xD406);

		OAL_IRQ_RESTORE;		
	}

     
      
    return;
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_freq_set_8242(UINT32 u32_freq)
{
    UINT64 u64_freq_setd;
    UINT64 u64_freq_setu;
    STATIC UINT32 s_u32_freq_ind = 0;
	UINT32 u32_fsd = 45;//1474560KHz = 32768 * 45KHz
	UINT32 u32_fsu = 135;//4423680KHz = 32768 * 135KHz
	UINT32 u32_i = 0;
	UINT32 u32_freq_local_ul = u32_freq;
	UINT32 u32_freq_local_dl = u32_freq;	
	UINT16 u16_freq_reg[14][4] = {
		{0x80,0x000,0x00,0x001},
		{0x80,0x216,0x39,0x001},
		{0x80,0x217,0xDE,0x001},
		{0x80,0x218,0xD0,0x001},
		{0x80,0x219,0x97,0x001},
		{0x80,0x21A,0xB4,0x001},
		{0x80,0x21B,0x25,0x001},
		//{0x80,0x209,0x00,0x001},
		//{0x80,0x208,0x87,0x001},
		{0x80,0x700,0x00,0x001},		
		{0x80,0xA16,0x39,0x001},
		{0x80,0xA17,0xDE,0x001},
		{0x80,0xA18,0xD0,0x001},
		{0x80,0xA19,0x97,0x001},
		{0x80,0xA1A,0xB4,0x001},
		{0x80,0xA1B,0x25,0x001},
		//{0x80,0xA09,0x00,0x001},
		//{0x80,0xA08,0x07,0x001},		
		};// RX IQ CAL

	#if 0
    if((1414000 < u32_freq) || (100000 > u32_freq))
    {
        return;
    }
    #endif
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC300 , (UINT16)(u32_freq >> 16));			
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC300 , (UINT16)u32_freq);	

	#if 0
	if(u32_freq_local_dl > FREQ_THRES)
	{
		//fdd
		u32_freq_local_dl -= FREQ_THRES;
		u32_freq_local_ul -= FREQ_THRES;		
		if((UINT8)g_zzw_node_status.lc_id & 1)
		{
			u32_freq_local_ul += FDD_DELTA_FREQ;
		}
		else
		{
			u32_freq_local_dl += FDD_DELTA_FREQ;
		}
	}
	#else
	if(g_zzw_freq_val_dl)
	{
		//fdd
		u32_freq_local_dl = g_zzw_freq_val_dl;
	}

	#endif

	u64_freq_setd = (((UINT64)u32_freq_local_dl << 33) / u32_fsd);
	//u64_freq_setd = (UINT64)u32_freq * u32_fsd;
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC310 , (UINT16)(u64_freq_setd >> 32));
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC310 , (UINT16)(u64_freq_setd >> 16));
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC310 , (UINT16)u64_freq_setd); 
	
	u64_freq_setu = (((UINT64)u32_freq_local_ul << 33) / u32_fsu);
	//u64_freq_setu = (UINT64)u32_freq * u32_fsu;
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC311 , (UINT16)(u64_freq_setu >> 32));
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC311 , (UINT16)(u64_freq_setu >> 16));
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC311 , (UINT16)u64_freq_setu);

	
    //s_u32_rf_freq = u32_freq;

    rf_sch_spi_status_check();

    rf_spi_AHB_control_dsp();
    //rf_spi_tx_num_config(1);

	for(u32_i = 0; u32_i < 6; u32_i++)
	{
	    u16_freq_reg[u32_i + 1][2] = (UINT16)((u64_freq_setd >> ((5 - u32_i) * 8)) & (0xff));
	    
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC301 , (UINT16)u16_freq_reg[u32_i][2]);			
    }

	for(u32_i = 0; u32_i < 6; u32_i++)
	{
	    u16_freq_reg[u32_i + 8][2] = (UINT16)((u64_freq_setu >> ((5 - u32_i) * 8)) & (0xff));
    
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC301 , (UINT16)u16_freq_reg[u32_i + 7][2]);			
    }


	rf_spi_config(RF_SPI_USED, 14, KS_RF_CXPS8242, u16_freq_reg); 

}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_freq_set_8242_single(UINT32 u32_freq)
{
    UINT64 u64_freq_setd;
    UINT64 u64_freq_setu;
    STATIC UINT32 s_u32_freq_ind = 0; //odd for dl ,  even for ul
	UINT32 u32_fsd = 1474560;
	UINT32 u32_fsu = 4423680;
	UINT32 u32_i = 0;
	UINT16 u16_freq_reg_dl[7][4] = {
		{0x80,0x000,0x00,0x001},
		{0x80,0x216,0x39,0x001},
		{0x80,0x217,0xDE,0x001},
		{0x80,0x218,0xD0,0x001},
		{0x80,0x219,0x97,0x001},
		{0x80,0x21A,0xB4,0x001},
		{0x80,0x21B,0x25,0x001}};
	UINT16 u16_freq_reg_ul[7][4] = {		
		//{0x80,0x209,0x00,0x001},
		//{0x80,0x208,0x87,0x001},
		{0x80,0x700,0x00,0x001},		
		{0x80,0xA16,0x39,0x001},
		{0x80,0xA17,0xDE,0x001},
		{0x80,0xA18,0xD0,0x001},
		{0x80,0xA19,0x97,0x001},
		{0x80,0xA1A,0xB4,0x001},
		{0x80,0xA1B,0x25,0x001},
		//{0x80,0xA09,0x00,0x001},
		//{0x80,0xA08,0x07,0x001},		
		};// RX IQ CAL

	#if 0
    if((1414000 < u32_freq) || (100000 > u32_freq))
    {
        return;
    }
    #endif
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC200 , (UINT16)(u32_freq >> 16));			
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC200 , (UINT16)u32_freq);	

	if((s_u32_freq_ind & 0x1) EQ 0)
	{
		//dl
		u64_freq_setd = (((UINT64)u32_freq << 40) / u32_fsd) << 8;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC210 , (UINT16)(u64_freq_setd >> 32));
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC210 , (UINT16)(u64_freq_setd >> 16));
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC210 , (UINT16)u64_freq_setd);	

		for(u32_i = 0; u32_i < 6; u32_i++)
		{
		    u16_freq_reg_dl[u32_i + 1][2] = (UINT16)((u64_freq_setd >> ((5 - u32_i) * 8)) & (0xff));
		    
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC201 , (UINT16)u16_freq_reg_ul[u32_i][2]);			
	    }

	    rf_sch_spi_status_check();
	    rf_spi_AHB_control_dsp();
	    //rf_spi_tx_num_config(1);
		rf_spi_config(RF_SPI_USED, 7, KS_RF_CXPS8242, u16_freq_reg_dl); 
	}
	else
	{	
		u64_freq_setu = (((UINT64)u32_freq << 40) / u32_fsu) << 8;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC211 , (UINT16)(u64_freq_setu >> 32));
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC211 , (UINT16)(u64_freq_setu >> 16));
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC211 , (UINT16)u64_freq_setu);	

		for(u32_i = 0; u32_i < 6; u32_i++)
		{
		    u16_freq_reg_ul[u32_i + 1][2] = (UINT16)((u64_freq_setu >> ((5 - u32_i) * 8)) & (0xff));
	    
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC201 , (UINT16)u16_freq_reg_dl[u32_i][2]);			
	    }
	    rf_sch_spi_status_check();
	    rf_spi_AHB_control_dsp();
	    //rf_spi_tx_num_config(1);
		rf_spi_config(RF_SPI_USED, 7, KS_RF_CXPS8242, u16_freq_reg_ul); 	    
	}
    s_u32_freq_ind &= 1;
}

#endif

RF_DRV_ITCM_CODE_SECTION
VOID rf_drv_init()
{
    UINT32 u32_rfctrl_irq_lev[16] = { OAL_IRQ_LVL_0, OAL_IRQ_LVL_2, OAL_IRQ_LVL_0, OAL_IRQ_LVL_2,
                                      OAL_IRQ_LVL_0, OAL_IRQ_LVL_2, OAL_IRQ_LVL_0, OAL_IRQ_LVL_2,
                                      OAL_IRQ_LVL_2, OAL_IRQ_LVL_2, OAL_IRQ_LVL_0, OAL_IRQ_LVL_2,
                                      OAL_IRQ_LVL_2, OAL_IRQ_LVL_2, OAL_IRQ_LVL_0, OAL_IRQ_LVL_2};

	#ifdef KD001_RF8242
	#if MIMO_2ANTS
    rf_rffe_init(KS_RF_CXPS8242, 24, 1, 0);
	#else
    rf_rffe_init(KS_RF_CXPS8242, 23, 1, 0);
	#endif	  
	#else
    rf_spi_init(KS_SPI_NOR, 0, 0, KS_RF_CXPS9261);
	#ifdef DVB
    ks_oal_set_reg(0x50e0078, 4, 6, 3);
    WRITE_REG32(0x10400040, ((READ_REG32(0x10400040))|((0x1<<20)|(0x1<<4))));
	WRITE_REG32(0x10400000, ((READ_REG32(0x10400000))|((0x1<<20)|(0x1<<4))));
	#endif
    rf_spi_config(RF_SPI_USED, 95, KS_RF_CXPS9261, rf_cx9261_cfg);

	//rf_spi_config(DD_SPI1, 76, KS_RF_CXPS9261, rf_cx9261_cfg);
    ks_oal_delay(RF_SPI_CONF_DELAY);
    
    rf_rffe_init(KS_RF_CXPS9261, 6, 0,1);
    rf_rffe_tx_af_config(4);
    rf_rffe_rx_aem_config(4);
    rf_rffe_IQ_swap_tx();
    rf_rffe_IQ_swap_rx();
	#endif


    #ifdef BAND_WIDTH_10M
    rf_ctrl_init(1, 39);
	#else
    rf_ctrl_init(1, 19);
	#endif
    rf_ctrl_frame_config(61440);
    rf_ctrl_interr_level_set(u32_rfctrl_irq_lev);
    rf_ctrl_isr_handle_register(0, rf_sch_evt_int, OAL_IRQ_LVL_0);
    rf_ctrl_isr_handle_register(4, rf_sch_frm_int, OAL_IRQ_LVL_0);

    rf_ctrl_interr_pos_config(0x1000, 2);
    rf_ctrl_interr_config(0x11, 0);
    rf_ctrl_interr_register(0x11);

    rf_ctrl_trigger_trace(16, 0);
	//rf_spi_init(KS_SPI_NOR, 1, 1, KS_RF_CXPS9261);

	#ifdef DVB
	ks_oal_set_reg(0x50e0078, 4, 6, 0);
	#endif

    s_u32_frm_int_cnt = 0;
}
/***********************************************************************************************************************
* FUNCTION  
*
*   rf_sch_tx_pow_set
*
* DESCRIPTION
*
* TDD tx power set
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   SINT32 s32_tx_pwr  dbm ((5dbm ~~ -36.5dbm) * 2)
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
* 2020/08/10          gjp
***********************************************************************************************************************/
#define AGC_NEW (1)
RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_gain_set_sch(UINT32_PTR u32p_evt_ram_addr)
{
	SINT32 s32_rx_gain;
    UINT32 u32_gain_idx;
    UINT8 u8_node_id;

	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);
	node_info_t *stp_node_info;


    *(u32p_evt_ram_addr + 1)  = 0x915F0008; /* agc initiation rf spi tri*/
    *(u32p_evt_ram_addr + 2) += 0x8;

    rf_spi_AHB_control_hw();
    rf_spi_tx_num_config(5);

	u8_node_id = node_slot_alloc_query((UINT8)g_zzw_ssap, g_s32_frame_no);
    #if AGC_NEW   
	if(u8_node_id)
	{
		if((stp_node_info = node_tbl_find(u8_node_id)))
		{
			//u8_node_id neq 0 and node info exists
			s32_rx_gain = (SINT8)(stp_node_info->u8_gain);
			if(s32_rx_gain < 0)
			{
				s32_rx_gain = 0;
			}
		    u32_gain_idx = ((s32_rx_gain/10) < 7) ? (s32_rx_gain/10) : 7;
		}
		else
		{
			u32_gain_idx = (SINT32)(((UINT32)g_s32_frame_no % 3) << 1);
		}
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC140 , (UINT16)u32_gain_idx);


		WRITE_REG32(0x17601200, 0x00F06D00 | s_u16_gain_reg[u32_gain_idx][0]);
        WRITE_REG32(0x17601204, 0x00F06D00 | s_u16_gain_reg[u32_gain_idx][0]);
        WRITE_REG32(0x17601208, 0x00F06C00 | s_u16_gain_reg[u32_gain_idx][1]);
        WRITE_REG32(0x1760120C, 0x00F05B00 | s_u16_gain_reg[u32_gain_idx][2]);
        WRITE_REG32(0x17601210, 0x00F05C00 | s_u16_gain_reg[u32_gain_idx][3]);	

		g_u8_rx_gain_idx = u32_gain_idx;		
        g_u8_gain_set_flg = (UINT8)OAL_TRUE;
	}
	else
	#endif
	{
		WRITE_REG32(0x17601200, 0x00F06D0F);
        WRITE_REG32(0x17601204, 0x00F06D0F);
        WRITE_REG32(0x17601208, 0x00F06C14);
        WRITE_REG32(0x1760120C, 0x00F05B3F);
        WRITE_REG32(0x17601210, 0x00F05C60);
	    g_u8_gain_set_flg = (UINT8)OAL_FALSE;
	}
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_gain_set(SINT32 s32_rx_gain)
{
    UINT32 u32_gain_idx;
    UINT8 u8_node_id;
    STATIC UINT16 s_u16_gain_reg[8][4] = {{0x0F, 0x00, 0x3F, 0x60},  /*01db*/
                                          {0x1F, 0x0C, 0x3F, 0x60},  /*10db*/
                                          {0x3F, 0x1C, 0x3F, 0x60},  /*20db*/
                                          {0x3F, 0x2D, 0x3F, 0x60},  /*30db*/
                                          {0x3F, 0x27, 0x3F, 0x60},  /*40db*/
                                          {0x3F, 0x2F, 0x7F, 0x62},  /*50db*/
                                          {0x3F, 0x2F, 0xFF, 0x60},  /*60db*/
                                          {0x3F, 0x2F, 0xFF, 0x65}}; /*65db*/
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);
	node_info_t *stp_node_info;


	if(!g_u8_gain_set_flg)
	{
	    s32_rx_gain = (s32_rx_gain <= 0) ? 1 : s32_rx_gain;

	    u32_gain_idx = ((s32_rx_gain/10) < 7) ? (s32_rx_gain/10) : 7;

	    rf_sch_spi_status_check();

	    rf_spi_AHB_control_dsp();
	    rf_spi_tx_num_config(1);
		
	    rf_spi_write_reg(RF_SPI_USED, 0x6D, s_u16_gain_reg[u32_gain_idx][0], KS_RF_CXPS9261);
	    rf_spi_en();
	    
	    rf_sch_spi_status_check();

	    rf_spi_write_reg(RF_SPI_USED, 0x6D, s_u16_gain_reg[u32_gain_idx][0], KS_RF_CXPS9261);
	    rf_spi_en();

	    rf_sch_spi_status_check();

	    rf_spi_write_reg(RF_SPI_USED, 0x6C, s_u16_gain_reg[u32_gain_idx][1], KS_RF_CXPS9261);
	    rf_spi_en();

	    rf_sch_spi_status_check();

	    rf_spi_write_reg(RF_SPI_USED, 0x5B, s_u16_gain_reg[u32_gain_idx][2], KS_RF_CXPS9261);
	    rf_spi_en();

	    rf_sch_spi_status_check();

	    rf_spi_write_reg(RF_SPI_USED, 0x5C, s_u16_gain_reg[u32_gain_idx][3], KS_RF_CXPS9261);
	    rf_spi_en();
	    rf_sch_spi_status_check();
    }
    else
    {
	    g_u8_gain_set_flg = OAL_FALSE;
	    u32_gain_idx = g_u8_rx_gain_idx;
    }
    
    
    if(0 == u32_gain_idx)
    {
        stp_sarm_1643_4500->u8_rx_gain = 1;
    }
    else if(7 == u32_gain_idx)
    {
        stp_sarm_1643_4500->u8_rx_gain = 65;
    }
    else
    {
        stp_sarm_1643_4500->u8_rx_gain = (UINT8)u32_gain_idx*10;
    }
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC201 , (UINT16)stp_sarm_1643_4500->u8_rx_gain);	

}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_evt_cfg(UINT32_PTR u32p_evt_table_addr, UINT32 u32_evt_table_size, UINT32_PTR u32p_evt_ram_addr, UINT32 u32_next_op)
{
    SINT32 s32_index;
    STATIC SINT32 s32_tim_adj_value = 0;
    rf_ctrl_event_ie_t* stp_tim_adj_evt;

    OAL_ASSERT(((NULL_PTR != u32p_evt_table_addr) && (NULL_PTR != u32p_evt_ram_addr)), "evt_pointer is null");
    OAL_ASSERT((0 != u32_evt_table_size), "u32_evt_table_size is 0");

    /*write event into rf_sch ram*/
    ks_oal_mem_copy(u32p_evt_ram_addr, u32p_evt_table_addr, u32_evt_table_size);

    /*timing adjustment*/
    if(OAL_TRUE == g_zzw_rtc.u32_valid)
    { 
        s32_tim_adj_value = g_zzw_rtc.st_rtc.s32_rtc_value * 2; /*将rtc值换算为rf_sch cycle, rtc正值表示缩短帧，负值表示拉长帧*/
        s32_tim_adj_value++;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC111 , (UINT16)s32_tim_adj_value);			

        if(NOP == u32_next_op) 
        {
            /*缩帧*/
            if(s32_tim_adj_value > 0)
            {
                s32_index = (u32_evt_table_size>>2) - 6;
            }
            else /*扩帧*/
            {
                s32_index = (u32_evt_table_size>>2) - 1;
            }

            /*将tim_adj event作为最后一个event写入rf_sch ram*/
            stp_tim_adj_evt = (rf_ctrl_event_ie_t*)&(u32p_evt_ram_addr[s32_index]);
            stp_tim_adj_evt->b16_evt_ext_tim = (s32_tim_adj_value & 0xffff);
            stp_tim_adj_evt->b8_evt_id       = (s32_tim_adj_value>>16) & 0xff;
            stp_tim_adj_evt->b2_evt_trig_typ = 0;
            stp_tim_adj_evt->b1_evt_slot_lim = 0;
            stp_tim_adj_evt->b1_evt_slot_adj = 1;
            stp_tim_adj_evt->b1_evt_tim_typ  = 0;
            stp_tim_adj_evt->b1_evt_int_en   = 0;
            stp_tim_adj_evt->b1_evt_last     = 1;
            stp_tim_adj_evt->b1_evt_valid    = 1;
        }
        else
        {
            if(RX == u32_next_op)
            {
                stp_tim_adj_evt = (rf_ctrl_event_ie_t*)&(u32p_evt_ram_addr[(u32_evt_table_size>>2) - 4]);
                //u32p_evt_ram_addr[(u32_evt_table_size>>2) - 3] += 0x120;
            }
            else
            {
        		#ifdef MIMO2
                stp_tim_adj_evt = (rf_ctrl_event_ie_t*)&(u32p_evt_ram_addr[(u32_evt_table_size>>2) - 4]);
        		#else
                stp_tim_adj_evt = (rf_ctrl_event_ie_t*)&(u32p_evt_ram_addr[(u32_evt_table_size>>2) - 5]);
                #endif
                //u32p_evt_ram_addr[(u32_evt_table_size>>2) - 5] += 0x120;
            }
           
            stp_tim_adj_evt->b16_evt_ext_tim = (s32_tim_adj_value & 0xffff);
            stp_tim_adj_evt->b8_evt_id       = (s32_tim_adj_value>>16) & 0xff;
            stp_tim_adj_evt->b2_evt_trig_typ = 0;
            stp_tim_adj_evt->b1_evt_slot_lim = 0;
            stp_tim_adj_evt->b1_evt_slot_adj = 1;
            stp_tim_adj_evt->b1_evt_tim_typ  = 0;
            stp_tim_adj_evt->b1_evt_int_en   = 0;
            stp_tim_adj_evt->b1_evt_last     = 1;
            stp_tim_adj_evt->b1_evt_valid    = 1;

            if(RX == u32_next_op)
            {
                u32p_evt_ram_addr[(u32_evt_table_size>>2) - 2] = 0x914C0008; /*config rxdfe sync event*/
                #if MIMO_2ANTS
				u32p_evt_ram_addr[(u32_evt_table_size>>2) - 3] = 0x91510008; /*config rxdfe sync event*/
				#endif
               
                s_u32_rxdfe_sync_event_addr = (UINT32)&u32p_evt_ram_addr[(u32_evt_table_size>>2) - 2];

                if(g_zzw_rtc.st_rtc.s32_rtc_value < 0) /*rx扩帧*/
                {
                    g_rxdfe_timing_ext_indication = OAL_TRUE;
                }
            }

            g_dfe_timing_info.u32_timing_update_valid      = OAL_TRUE;
            g_dfe_timing_info.u32_timing_update_frame_type = u32_next_op;
            g_dfe_timing_info.s32_timing_update_value      = g_zzw_rtc.st_rtc.s32_rtc_value;
        }
        #ifdef FUN_OTD
		if(g_s16_otd_flg EQ 1)
		{
			g_s16_otd_flg = 2;
		}
		#endif
        g_zzw_rtc.u32_valid = OAL_FALSE;
    }
    else
    {
		#if 0//def MIMO2
	    if(g_zzw_freq_set_flag EQ OAL_TRUE)
	    {
			KS_LOG(0, 0XDDDB, g_zzw_freq_val);
			KS_LOG(0, 0XDDDB, (u32_evt_table_size >> 2));
			if(RFOP_2_TYPE(u32_next_op) EQ TX)
			{
				rf_sch_freq_set_sch(g_zzw_freq_val , (u32_evt_table_size >> 2) - 4, g_u16_freq_reg, u32p_evt_ram_addr);
			}
			else
			{
				rf_sch_freq_set_sch(g_zzw_freq_val , (u32_evt_table_size >> 2) - 10, g_u16_freq_reg, u32p_evt_ram_addr);				
			}
			g_zzw_freq_set_flag = OAL_FALSE;
	    }
	    #endif    
    }
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_rx2rx(UINT32_PTR u32p_evt_ram_addr)
{
    rf_sch_evt_cfg(s_u32_event_table_rx2rx, sizeof(s_u32_event_table_rx2rx), u32p_evt_ram_addr, RX);
    if((1 == g_zzw_freq_scan_flag) && (0 == g_zzw_freq_scan_star))
    {
        *(u32p_evt_ram_addr + 8)  = 0x80000100; /* agc initiation rf spi tri*/
        *(u32p_evt_ram_addr + 16) = 0xA0000008;
    }
    else if((g_zzw_node_status.state >= ZZW_STATE_WAN) && (0xFF == g_zzw_rx_gain))
    {
        #if RF_SCH_SPI_ENABLE
		rf_sch_gain_set_sch(u32p_evt_ram_addr);
        #endif
    }
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_rx2tx(UINT32_PTR u32p_evt_ram_addr)
{
    /*if current frame is the last rx frame, cancel rxdfe_sync event*/
    if((OAL_FALSE == g_rxdfe_timing_ext_indication) && (0 != s_u32_rxdfe_sync_event_addr))
    {
        WRITE_REG32(s_u32_rxdfe_sync_event_addr, 0x80000008);
        WRITE_REG32(s_u32_rxdfe_sync_event_addr - 4, 0x80000008);

        s_u32_rxdfe_sync_event_addr = 0;
    }

    g_rxdfe_timing_ext_indication = OAL_FALSE;

    /*reset txdfe*/
    txdfe_start(0);
    /*clear tx rffe fifo*/
    rf_rffe_tx_rst();

    rf_sch_evt_cfg(s_u32_event_table_rx2tx, sizeof(s_u32_event_table_rx2tx), u32p_evt_ram_addr, TX);
    #ifdef KD001_RF8242
    #else
    //rf_spi_pwr_set(u32p_evt_ram_addr);
	#endif
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_rx2nop(UINT32_PTR u32p_evt_ram_addr)
{
    /*if current frame is the last rx frame, cancel rxdfe_sync event*/
    if((OAL_FALSE == g_rxdfe_timing_ext_indication) && (0 != s_u32_rxdfe_sync_event_addr))
    {
        WRITE_REG32(s_u32_rxdfe_sync_event_addr, 0x80000008);
        WRITE_REG32(s_u32_rxdfe_sync_event_addr - 4, 0x80000008);

        s_u32_rxdfe_sync_event_addr = 0;
    }

    g_rxdfe_timing_ext_indication = OAL_FALSE;

    rf_sch_evt_cfg(s_u32_event_table_rx2nop, sizeof(s_u32_event_table_rx2nop), u32p_evt_ram_addr, NOP);
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_tx2rx(UINT32_PTR u32p_evt_ram_addr)
{
    /*reset rxdfe*/
    rxdfe_start(0);
    /*clear rx rffe fifo*/
    rf_rffe_rx_rst();

    rf_sch_evt_cfg(s_u32_event_table_tx2rx, sizeof(s_u32_event_table_tx2rx), u32p_evt_ram_addr, RX);

    if((g_zzw_node_status.state >= ZZW_STATE_WAN) && (0xFF == g_zzw_rx_gain))
    {
        #if RF_SCH_SPI_ENABLE
		rf_sch_gain_set_sch(u32p_evt_ram_addr);
        #endif
    }

    /*当前帧为last tx帧*/
    framing_last_is_cfg(1, 0);
    #if MIMO_2ANTS
	framing_last_is_cfg(1, 1);
	#endif
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_tx2tx(UINT32_PTR u32p_evt_ram_addr)
{
    rf_sch_evt_cfg(s_u32_event_table_tx2tx, sizeof(s_u32_event_table_tx2tx), u32p_evt_ram_addr, TX);

    #ifdef KD001_RF8242
    #else
    //rf_spi_pwr_set(u32p_evt_ram_addr);
	#endif
	#ifdef MIMO2
    framing_last_is_cfg(1, 0);
    #if MIMO_2ANTS
	framing_last_is_cfg(1, 1);
	#endif
	#endif
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_tx2nop(UINT32_PTR u32p_evt_ram_addr)
{
    rf_sch_evt_cfg(s_u32_event_table_tx2nop, sizeof(s_u32_event_table_tx2nop), u32p_evt_ram_addr, NOP);
    /*当前帧为last tx帧*/
    framing_last_is_cfg(1, 0);
    #if MIMO_2ANTS
	framing_last_is_cfg(1, 1);
	#endif
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_nop2rx(UINT32_PTR u32p_evt_ram_addr)
{
    /*reset rxdfe*/
    rxdfe_start(0);
    /*clear rx rffe fifo*/
    rf_rffe_rx_rst();

    rf_sch_evt_cfg(s_u32_event_table_nop2rx, sizeof(s_u32_event_table_nop2rx), u32p_evt_ram_addr, RX);
    if((1 == g_zzw_freq_scan_flag) && (0 == g_zzw_freq_scan_star))
    {
        *(u32p_evt_ram_addr + 11)  = 0x80000100; /* agc initiation rf spi tri*/
        *(u32p_evt_ram_addr + 19)  = 0xA0000008;
    }
    else if((g_zzw_node_status.state >= ZZW_STATE_WAN) && (0xFF == g_zzw_rx_gain))
    {
        #if RF_SCH_SPI_ENABLE
		rf_sch_gain_set_sch(u32p_evt_ram_addr);
        #endif
    }
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_nop2tx(UINT32_PTR u32p_evt_ram_addr)
{
    /*reset txdfe*/
    txdfe_start(0);
    /*clear tx rffe fifo*/
    rf_rffe_tx_rst();
    
    rf_sch_evt_cfg(s_u32_event_table_nop2tx, sizeof(s_u32_event_table_nop2tx), u32p_evt_ram_addr, TX);
    #ifdef KD001_RF8242
    #else
    //rf_spi_pwr_set(u32p_evt_ram_addr);
	#endif    
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_nop2nop(UINT32_PTR u32p_evt_ram_addr)
{
    rf_sch_evt_cfg(s_u32_event_table_nop, sizeof(s_u32_event_table_nop), u32p_evt_ram_addr, NOP);
}

RF_DRV_ITCM_CODE_SECTION
VOID dfe_timing_update()
{
    if(OAL_TRUE == g_dfe_timing_info.u32_frame_rst_flag)
    {
        framing_timing_update(-g_dfe_timing_info.s32_timing_rst_value, 4, 0);
		framing_timing_update(-g_dfe_timing_info.s32_timing_rst_value, 4, 1);
        g_dfe_timing_info.u32_frame_rst_flag = OAL_FALSE;
    }

    if(OAL_TRUE == g_dfe_timing_info.u32_timing_update_valid)
    {
        if(RX == g_dfe_timing_info.u32_timing_update_frame_type)
        {
            if(g_dfe_timing_info.s32_timing_update_value > 0)
            {
                rxdfe_timing_update(g_dfe_timing_info.s32_timing_update_value, 0, 0); /*rxdfe output 1x*/
				rxdfe_timing_update(g_dfe_timing_info.s32_timing_update_value, 0, 1); /*rxdfe output 1x*/
            }
            else
            {
                rxdfe_timing_update(-g_dfe_timing_info.s32_timing_update_value, 1, 0); /*rxdfe output 1x*/
				rxdfe_timing_update(-g_dfe_timing_info.s32_timing_update_value, 1, 1); /*rxdfe output 1x*/
            }
        }
        else if(TX == g_dfe_timing_info.u32_timing_update_frame_type)
        {
            /*rtc正值表示缩短帧，负值表示拉长帧*/
            framing_timing_update(g_dfe_timing_info.s32_timing_update_value, 4, 0);
            g_dfe_timing_info.u32_frame_rst_flag   = OAL_TRUE;
            g_dfe_timing_info.s32_timing_rst_value = g_dfe_timing_info.s32_timing_update_value;
        }

        g_dfe_timing_info.u32_timing_update_valid = OAL_FALSE;
    }
}

/***********************************************************************************************************************
* FUNCTION  
*
*   rf_sch_freq_set
*
* DESCRIPTION
*
* TDD tx rx freq set
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   UINT32 u32_freq  khz
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
* 2020/08/10          gjp
***********************************************************************************************************************/
RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_spi_status_check()
{
    //return;

    while(((READ_REG32(pRFIF_SPI_EN)) & ((1<<SPI_EN_SPI_EN) | (1<<SPI_TP_SPI_ACTIVE))) != 0)
    {
        g_u32_spi_wait_cnt++;
    }
}
RF_DRV_ITCM_CODE_SECTION
eKS_CX9261_LO_FREQ_RANGE rf_sch_cx9261_lo_get(UINT32 u32_freq)
{
    if(u32_freq <= 2700000)
    {
        if(u32_freq >= 1900000)
        {
            return CX9261_LO_FREQ_RANGE_2700MHZ_TO_1900MHz;
        }
        else if(u32_freq >= 1350000)
        {
            return CX9261_LO_FREQ_RANGE_1900MHZ_TO_1350MHz;
        }
        else if(u32_freq >= 950000)
        {
            return CX9261_LO_FREQ_RANGE_1350MHZ_TO_950MHz;
        }
        else if(u32_freq >= 675000)
        {
            return CX9261_LO_FREQ_RANGE_950MHZ_TO_675MHz;
        }
        else if(u32_freq >= 475000)
        {
            return CX9261_LO_FREQ_RANGE_675MHZ_TO_475MHz;
        }
        else if(u32_freq >= 337500)
        {
            return CX9261_LO_FREQ_RANGE_475MHZ_TO_337p5MHz;
        }
        else if(u32_freq >= 237500)
        {
            return CX9261_LO_FREQ_RANGE_337p5MHZ_TO_237p5MHz;
        }
        else if(u32_freq >= 168750)
        {
            return CX9261_LO_FREQ_RANGE_237p5MHZ_TO_168p75MHz;
        }
        else if(u32_freq >= 118750)
        {
            return CX9261_LO_FREQ_RANGE_168p75MHZ_TO_118p75MHz;
        }
        else if(u32_freq >= 84375)
        {
            return CX9261_LO_FREQ_RANGE_118p75MHZ_TO_84p375MHz;
        }
        else if(u32_freq >= 59375)
        {
            return CX9261_LO_FREQ_RANGE_84p375MHZ_TO_59p375MHz;
        }
        else if(u32_freq >= 42187)
        {
            return CX9261_LO_FREQ_RANGE_59p375MHZ_TO_42p1875MHz;
        }
        else
        {
            return CX9261_LO_FREQ_RANGE_ERROR;
        }
    }
    else
    {
        return CX9261_LO_FREQ_RANGE_ERROR;
    }
}
#define CX9261_FREQ_FILTER_Q36 (2863311531u)   //  2^36*(1div24)
RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_freq_set(UINT32 u32_freq , UINT16 (*u16_freq_reg)[4])
{
    UINT64 u64_freq, u64_freq_high, u64_freq_low;
    eKS_CX9261_LO_FREQ_RANGE lo_freq_range;

    lo_freq_range = rf_sch_cx9261_lo_get(u32_freq);

    if(CX9261_LO_FREQ_RANGE_ERROR == lo_freq_range)
    {
        return;
    }


    u64_freq_high = u32_freq / 1000;
    u64_freq_low = u32_freq % 1000;

    u64_freq = u64_freq_high * CX9261_FREQ_FILTER_Q36 + (u64_freq_low * CX9261_FREQ_FILTER_Q36) / 1000000;
    u64_freq = u64_freq << cx9261_vco_map[lo_freq_range][1];
    u64_freq = u64_freq >> 11;

    rf_sch_spi_status_check();
    rf_spi_AHB_control_dsp();
    rf_spi_tx_num_config(1);

    u16_freq_reg[2][2] = (u16_freq_reg[2][2] & (~(1<<4))) | (cx9261_vco_map[lo_freq_range][0] << 4);
    u16_freq_reg[1][2] = (u16_freq_reg[1][2] & (~(7<<0))) | (cx9261_vco_map[lo_freq_range][1] << 0);

    u16_freq_reg[0][2] = (UINT16)((u64_freq >> 25) & (0xff));
    u16_freq_reg[2][2] = (UINT16)(u16_freq_reg[2][2] | (((u64_freq >> 0) & 0x1)) << 7);
    u16_freq_reg[3][2] = (UINT16)((u64_freq >> 1)  & (0xff));
    u16_freq_reg[4][2] = (UINT16)((u64_freq >> 9) & (0xff));
    u16_freq_reg[5][2] = (UINT16)((u64_freq >> 17) & (0xff));

    rf_spi_config(RF_SPI_USED, CX9261_FREQ_SET_SPI_NUM, KS_RF_CXPS9261, u16_freq_reg);

}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_freq_set_sch(UINT32 u32_freq , UINT32 u32_pos, UINT16 (*u16_freq_reg)[4], UINT32_PTR u32p_evt_ram_addr)
{
    UINT64 u64_freq, u64_freq_high, u64_freq_low;
    eKS_CX9261_LO_FREQ_RANGE lo_freq_range;

    lo_freq_range = rf_sch_cx9261_lo_get(u32_freq);

    if(CX9261_LO_FREQ_RANGE_ERROR == lo_freq_range)
    {
        return;
    }


    u64_freq_high = u32_freq / 1000;
    u64_freq_low = u32_freq % 1000;

    u64_freq = u64_freq_high * CX9261_FREQ_FILTER_Q36 + (u64_freq_low * CX9261_FREQ_FILTER_Q36) / 1000000;
    u64_freq = u64_freq << cx9261_vco_map[lo_freq_range][1];
    u64_freq = u64_freq >> 11;

    u16_freq_reg[2][2] = (u16_freq_reg[2][2] & (~(1<<4))) | (cx9261_vco_map[lo_freq_range][0] << 4);
    u16_freq_reg[1][2] = (u16_freq_reg[1][2] & (~(7<<0))) | (cx9261_vco_map[lo_freq_range][1] << 0);

    u16_freq_reg[0][2] = (UINT16)((u64_freq >> 25) & (0xff));
    u16_freq_reg[2][2] = (UINT16)(u16_freq_reg[2][2] | (((u64_freq >> 0) & 0x1)) << 7);
    u16_freq_reg[3][2] = (UINT16)((u64_freq >> 1)  & (0xff));
    u16_freq_reg[4][2] = (UINT16)((u64_freq >> 9) & (0xff));
    u16_freq_reg[5][2] = (UINT16)((u64_freq >> 17) & (0xff));
    
    *(u32p_evt_ram_addr + u32_pos)  = 0x915F0008; /* agc initiation rf spi tri*/

    rf_spi_AHB_control_hw();
    rf_spi_tx_num_config(CX9261_FREQ_SET_SPI_NUM);
	rf_spi_config_rfctrl(RF_SPI_USED, CX9261_FREQ_SET_SPI_NUM, KS_RF_CXPS9261, u16_freq_reg);
}


/***********************************************************************************************************************
* FUNCTION  
*
*   rf_sch_tx_pow_set
*
* DESCRIPTION
*
* TDD tx power set
*
* NOTE
*
*   <the limitations to use this function or other comments>
*
* PARAMETERS
*
*   SINT32 s32_tx_pwr  dbm ((5dbm ~~ -36.5dbm) * 2)
*
* RETURNS
*
*   NULL
*
* VERSION
*
*   <DATE>           <AUTHOR>           <CR_ID>              <DESCRIPTION>
* 2020/08/10          gjp
***********************************************************************************************************************/
RF_DRV_ITCM_CODE_SECTION
VOID rf_pwr_set(SINT16 s16_tx_pwr)
{
    STATIC SINT16 s_s16_rf_pwr = 0xFF;
    STATIC SINT32 s_s32_pwr_high[7] = {0xE1, 0x61, 0x21, 0x19, 0x11, 0x09, 0x01};
    SINT32 s32_idx, s32_pwr_low;

    if(s_s16_rf_pwr == s16_tx_pwr)
    {
        return;
    }

    if((-73 > s16_tx_pwr) || (10 < s16_tx_pwr))
    {
        return;
    }

    s_s16_rf_pwr = s16_tx_pwr;

    if(((READ_REG32(pRFIF_SPI_EN)) & ((1 << SPI_EN_SPI_EN) | (1 << SPI_TP_SPI_ACTIVE))) != 0)
    {
        OAL_ASSERT(0, "spi ctrl error");
    }

    rf_sch_spi_status_check();

    for(s32_idx = 0; s32_idx < 7; s32_idx++)
    {
        if((s16_tx_pwr + 12 * s32_idx) > -2)
        {
            s32_pwr_low = (s16_tx_pwr + 12 * s32_idx) + 1;
            break;
        }
    }

#if 1

    rf_spi_AHB_control_dsp();
    rf_spi_tx_num_config(1);

    rf_spi_write_reg(RF_SPI_USED, 0x83, ((s32_pwr_low << 4) | 4), KS_RF_CXPS9261);
    rf_spi_en();

    rf_sch_spi_status_check();

    rf_spi_write_reg(RF_SPI_USED, 0x83, ((s32_pwr_low << 4) | 4), KS_RF_CXPS9261);
    rf_spi_en();

    rf_sch_spi_status_check();

    rf_spi_write_reg(RF_SPI_USED, 0x85, s_s32_pwr_high[s32_idx], KS_RF_CXPS9261);
    rf_spi_en();

    rf_sch_spi_status_check();
#else

    g_rf_spi_ram[g_rf_spi_ram_idx++] = 0x00F08300 | ((s32_pwr_low << 4) | 4);
    g_rf_spi_ram[g_rf_spi_ram_idx++] = 0x00F08300 | ((s32_pwr_low << 4) | 4);
    g_rf_spi_ram[g_rf_spi_ram_idx++] = 0x00F08500 | s_s32_pwr_high[s32_idx];

#endif
}
RF_DRV_ITCM_CODE_SECTION
VOID rf_pwr_set_sch(UINT32_PTR u32p_evt_ram_addr)
{
	SINT32 s_s32_pwr_high[7] = {0xE1, 0x61, 0x21, 0x19, 0x11, 0x09, 0x01};
	SINT32 s32_idx, s32_pwr_low;
    UINT8 u8_node_id;
    SINT8 s8_tx_pwr =  g_zzw_tx_pwr & 0xFF;

	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);
	node_info_t *stp_node_info;

	#if MIMO_2ANTS
    *(u32p_evt_ram_addr + 6)  = 0x915F0008; /* rf spi tri*/
    //*(u32p_evt_ram_addr + 6) += 0x8;
	#else
    *(u32p_evt_ram_addr + 4)  = 0x915F0008; /* rf spi tri*/
    //*(u32p_evt_ram_addr + 4) += 0x8;
	#endif
	#if 1
    rf_spi_AHB_control_hw();
    rf_spi_tx_num_config(3);

       
	//u8_node_id = node_slot_alloc_query((UINT8)g_zzw_ssap, g_s32_frame_no);

	#if 1//APC_EN
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC150 , (UINT16)g_u8_node_id_target);

	#if APC_TEST
	if(s_st_node_info_tbl.u8_is_valid)
	{
		stp_node_info = &s_st_node_info_tbl.st_node_info[0];
		s8_tx_pwr = stp_node_info->s8_tx_pwr;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC151 , (UINT16)s8_tx_pwr);
	}
	#else
	if(g_zzw_tx_pwr EQ 0xFF)
	{
		if(g_u8_node_id_target && (stp_node_info = node_tbl_find(g_u8_node_id_target)))
		{
			//u8_node_id neq 0 and node info exists
			s8_tx_pwr = (SINT8)(stp_node_info->s8_tx_pwr);
			if(s8_tx_pwr EQ TX_PWR_INVALID)
			{
				s8_tx_pwr = (SINT8)(stp_node_info->s8_tx_pwr_expect);
			}
		}
		else
		{
			s8_tx_pwr = -60;
		}
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC151 , (UINT16)s8_tx_pwr);
	}
	#endif
	#endif
	if(s8_tx_pwr EQ TX_PWR_INVALID)
	{
	    s8_tx_pwr =  g_zzw_tx_pwr & 0xFF;
	}
    if(-73 > s8_tx_pwr)
    {
        s8_tx_pwr = -73;
    }

    if(0 < s8_tx_pwr)
    {
        s8_tx_pwr = 0;
    }

    for(s32_idx = 0; s32_idx < 7; s32_idx++)
    {
        if((s8_tx_pwr + 12 * s32_idx) > -2)
        {
            s32_pwr_low = (s8_tx_pwr + 12 * s32_idx) + 1;
            break;
        }
    }

    WRITE_REG32(0x17601200, 0x00F08300 | (((s32_pwr_low << 4) | 4) & 0xFF));
    WRITE_REG32(0x17601204, 0x00F08300 | (((s32_pwr_low << 4) | 4) & 0xFF));
    WRITE_REG32(0x17601208, 0x00F08500 | (s_s32_pwr_high[s32_idx] & 0xFF));
    #endif
}
RF_DRV_ITCM_CODE_SECTION
VOID rx_dfe_rssi_get_scan(UINT32 *u32p_rssi_avg, UINT8 u8_chid)
{
    UINT32 u32_idx;
	UINT32 u32_rssi_ram[8] = {0,0,0,0,0,0,0,0};
	
    rxdfe_intc_rpt_msk(0x8, u8_chid);
    ks_oal_mem_copy(u32_rssi_ram, (VOID_PTR)(REG_RXDFE_FK216_0 + 0x00040000 * u8_chid), 32);
    rxdfe_intc_rpt_clr(0x8, u8_chid);
    rxdfe_intc_rpt_msk(0x0, u8_chid);

    for(u32_idx = 0; u32_idx < 4; u32_idx++)
    {
        u32p_rssi_avg[0] += (u32_rssi_ram[u32_idx]>>2);

    }
	for(u32_idx = 4; u32_idx < 8; u32_idx++)
    {
        u32p_rssi_avg[1] += (u32_rssi_ram[u32_idx]>>2);

    }
}

RF_DRV_ITCM_CODE_SECTION
VOID rx_dfe_rssi_get_agc(UINT32 *u32p_rssi_avg, UINT8 u8_chid)
{
    UINT32 u32_idx;
	UINT32 u32_rssi_ram[8] = {0,0,0,0,0,0,0,0};
	
    rxdfe_intc_rpt_msk(0x8, u8_chid);
    ks_oal_mem_copy(u32_rssi_ram, (VOID_PTR)(REG_RXDFE_FK216_0 + 0x00040000 * u8_chid), 32);
    rxdfe_intc_rpt_clr(0x8, u8_chid);
    rxdfe_intc_rpt_msk(0x0, u8_chid);

	for(u32_idx = 0; u32_idx < 8; u32_idx++)
    {
        //u32_rssi_avg += (u32_rssi_ram[u32_idx]>>3);

        if(u32p_rssi_avg[0] < u32_rssi_ram[u32_idx])
        {
            u32p_rssi_avg[0] = u32_rssi_ram[u32_idx];
        }
    }
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_evt_int()
{
    UINT32 u32_rssi_avg[2][2] = {0};
    SINT32 s32_rx_gain = 1;
	STATIC UINT32 s_u32_rssi_rpt = 0;
    STATIC SINT32 s32_rssi_db[8] = {0};
	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);
	UINT8 u8_node_id = 0;

    KS_LOG_TMRL(0xC120);
	
	#ifdef MIMO2
    if((TX EQ RFOP_2_TYPE(g_u32_current_op)) && (TX == RFOP_2_TYPE(g_u32_next_op)))
    {
		/*curr event int is tx int , reset tx*/
		txdfe_start(0);
		rf_rffe_tx_rst();
		g_u8_evt_int_cnt = 0;
	    KS_LOG_TMRL(0xC122);

		return;
    }
    else if((TX EQ RFOP_2_TYPE(g_u32_current_op)) && (RX == RFOP_2_TYPE(g_u32_next_op)) && (g_u8_evt_int_cnt EQ 0))
    {
    	txdfe_start(0);
		rf_rffe_tx_rst();
		g_u8_evt_int_cnt += 1;
	    KS_LOG_TMRL(0xC123);

		return;
    }
    #endif
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC200 , (UINT16)g_zzw_rx_gain);	

    if((1 == g_zzw_freq_scan_flag) && (0 == g_zzw_freq_scan_star))
    {
        if(g_u32_rssi_cnt & 0x1)
        {
            rx_dfe_rssi_get_scan(u32_rssi_avg[0], 0);
			rx_dfe_rssi_get_scan(u32_rssi_avg[1], 1);

			if(u32_rssi_avg[0][0] > u32_rssi_avg[1][0])
			{
			    s32_rssi_db[s_u32_rssi_rpt]    = (10*(ks_logx(10, u32_rssi_avg[0][0]<<7)>>8))>>18;
                s32_rssi_db[s_u32_rssi_rpt++] -= stp_sarm_1643_4500->u8_rx_gain;
			}
			else
			{
			    s32_rssi_db[s_u32_rssi_rpt]    = (10*(ks_logx(10, u32_rssi_avg[1][0]<<7)>>8))>>18;
                s32_rssi_db[s_u32_rssi_rpt++] -= stp_sarm_1643_4500->u8_rx_gain;
			}

			if(u32_rssi_avg[0][1] > u32_rssi_avg[1][1])
			{
			    s32_rssi_db[s_u32_rssi_rpt]    = (10*(ks_logx(10, u32_rssi_avg[0][1]<<7)>>8))>>18;
                s32_rssi_db[s_u32_rssi_rpt++] -= stp_sarm_1643_4500->u8_rx_gain;
			}
			else
			{
			    s32_rssi_db[s_u32_rssi_rpt]    = (10*(ks_logx(10, u32_rssi_avg[1][1]<<7)>>8))>>18;
                s32_rssi_db[s_u32_rssi_rpt++] -= stp_sarm_1643_4500->u8_rx_gain;
			}

			if((s_u32_rssi_rpt & 0x7) == 0)
			{
			    oal_msg_t *stp_msg;
	            UINT32_PTR u32p_temp = NULL_PTR;

	            stp_msg = ks_oal_msg_create(16);
	            OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure");
	            stp_msg->msg_primitive_id = MSG_DL_RSSI_REPORT_PRIMITIVE_ID;
	            u32p_temp = (UINT32_PTR)&stp_msg->vp_msg_body;

				ks_oal_mem_copy(u32p_temp, s32_rssi_db, 32);
                ks_oal_msg_send(stp_msg, OAL_SYS_TASK_ID + 3);
                s_u32_rssi_rpt = 0;
			}
            
        }
        g_u32_rssi_cnt++;
    }
    else
    {
        /*Enable Auto Gain Control*/
        if(0xFF == g_zzw_rx_gain)
        {
            if(g_zzw_node_status.state >= ZZW_STATE_WAN)
            {
        		u8_node_id = node_slot_alloc_query((UINT8)g_zzw_ssap, g_s32_frame_no);
    		    #if AGC_NEW   
            	if(u8_node_id)
            	{
            		//do nothing
    			    if(0 == g_u8_rx_gain_idx)
				    {
				        stp_sarm_1643_4500->u8_rx_gain = 1;
				    }
				    else if(7 == g_u8_rx_gain_idx)
				    {
				        stp_sarm_1643_4500->u8_rx_gain = 65;
				    }
				    else
				    {
				        stp_sarm_1643_4500->u8_rx_gain = (UINT8)g_u8_rx_gain_idx*10;
				    }
            	}
            	else
            	#endif
            	{
	                rx_dfe_rssi_get_agc(u32_rssi_avg[0], 0);
					rx_dfe_rssi_get_agc(u32_rssi_avg[1], 1);

					if(u32_rssi_avg[0][0] > u32_rssi_avg[1][0])
					{
					    s32_rssi_db[0] = (10*(ks_logx(10, u32_rssi_avg[0][0]<<7)>>8))>>18;
	                    s32_rx_gain = ((SINT32)AGC_TARGET_RSSI_IN_DB - s32_rssi_db[0]) + 6;
					}
					else
					{
					    s32_rssi_db[0] = (10*(ks_logx(10, u32_rssi_avg[1][0]<<7)>>8))>>18;
	                    s32_rx_gain = ((SINT32)AGC_TARGET_RSSI_IN_DB - s32_rssi_db[0]) + 6;
					}

	                rf_sch_gain_set((UINT32)s32_rx_gain);            		
            	}
            }
            else
            {
                if(READ_REG32(RX_DEBUG_INFO_ADDR) <= 1)
                {
                    rf_sch_gain_set(g_u32_scan_gain_level*20);
                }
            }
        }
        else /*Enable Manual Gain Control*/
        {
            #ifdef KD001_RF8242

            #else
            rf_sch_gain_set(g_zzw_rx_gain);
            #endif
        }    
    }
    KS_LOG_TMRL(0xC121);
    g_u8_evt_int_cnt = 0;

}
RF_DRV_ITCM_CODE_SECTION VOID dfe_rtc_adjust_fast(SINT32 s32_rtc_value)
{

	if(s32_rtc_value > 15360)
	{
		//s32_frame_no  = (s32_frame_no + 1) & 0x7FF;
		s32_rtc_value -= 30720;
	}
	else if(s32_rtc_value < -15360)
	{
		//s32_frame_no  = (s32_frame_no - 1) & 0x7FF;
		s32_rtc_value += 30720;
	}

	g_zzw_rtc.u32_valid = OAL_TRUE;
	g_zzw_rtc.st_rtc.s32_frame_no = 0;
	g_zzw_rtc.st_rtc.s32_rtc_value = s32_rtc_value;
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC110 , (UINT16)s32_rtc_value); 
    #ifdef FUN_OTD
    g_s16_otd_flg = 1;
    #endif
    #if 0
	if(g_u32_1st_rtc_adj)
	{
    	ks_oal_mem_set(g_zzw_slot_table, 0x25, sizeof(g_zzw_slot_table));
    	g_u32_1st_rtc_adj = 0;

    	g_zzw_slot_table[0] = 0x24;
	}
	#endif

}

RF_DRV_ITCM_CODE_SECTION VOID dfe_rtc_adjust_fast2(SINT32 rtc_value)
{
	SINT32 s32_rtc_value = 0;
	g_zzw_rtc.u32_valid = OAL_TRUE;
	g_zzw_rtc.st_rtc.s32_frame_no = (SINT32)(rtc_value >> 16);
	s32_rtc_value = (SINT32)(rtc_value & 0xFFFF);

	if(s32_rtc_value > 32767)
	{
	   s32_rtc_value -= 65536;
	}
	g_zzw_rtc.st_rtc.s32_rtc_value = s32_rtc_value;

	
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC110 , (UINT16)((UINT32)rtc_value >> 16));    
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC110 , (UINT16)((UINT32)rtc_value));    
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC110 , (UINT16)((UINT32)s32_rtc_value>>16));
    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC110 , (UINT16)s32_rtc_value);    
   

	ks_oal_mem_set(g_zzw_slot_table, 0x25, sizeof(g_zzw_slot_table));
	g_u32_1st_rtc_adj = 0;
	g_u32_1st_frame = 1;

	g_zzw_slot_table[0] = 0x24;

    
}

RF_DRV_ITCM_CODE_SECTION
VOID rf_sch_frm_int()
{
    oal_msg_t *stp_msg = NULL_PTR;
    UINT32 u32_next_next_frm;
    UINT32_PTR u32p_evt_ram_addr = NULL_PTR;
    UINT32_PTR u32p_temp = NULL_PTR;
    //UINT32 u32_next_next_op_bak = g_u32_next2_op;
    //UINT32 u32_next3_op_bak = g_u32_next3_op;
    UINT32 u32_rf_regs[2];
    UINT32 u32_rf_reg_addr;

	sram_1643_4500_t *stp_sarm_1643_4500 = (sram_1643_4500_t*)(SRAM_1643_4500_ADDR);
	
    u32p_evt_ram_addr = (UINT32_PTR)(CP_SYS_INSTRUCTION_RAM_BASE + 0x400 * 4 * ((s_u32_frm_int_cnt + 1) & 0x3));
    KS_LOG_TMR(0xC102);
	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC103 , (UINT16)g_s32_frame_no);			

    /*帧中断计数*/
    if(OAL_TRUE == g_zzw_rtc.u32_valid)
    {
        g_s32_frame_no = (g_s32_frame_no + 1 - g_zzw_rtc.st_rtc.s32_frame_no);
        if(0 == g_zzw_rtc.st_rtc.s32_rtc_value)
        {
            g_zzw_rtc.u32_valid = OAL_FALSE;
        }
    }
    else
    {
        g_s32_frame_no = ((g_s32_frame_no + 1)); /*ZZW定义超帧为2048帧*/
    }
	u32_next_next_frm  = (g_s32_frame_no + 2) & 0x7FF;
	//ks_slp_proc();
    g_u32_current_op = g_u32_next_op;
	g_u32_next_op = g_u32_next2_op;
    g_u32_next2_op = g_u32_next3_op;
    if(g_s32_frame_no & 0x1)
    {
        //g_u32_next_op = g_zzw_slot_table[(g_s32_frame_no & 0x7FF) >> 1]>>4;
        //g_u32_next2_op = g_zzw_slot_table[((g_s32_frame_no + 1) & 0x7FF)>>1] & 0xF;
		g_u32_next3_op = g_zzw_slot_table[u32_next_next_frm>>1]>>4;
    }
    else
    {
        //g_u32_next_op = g_zzw_slot_table[(g_s32_frame_no & 0x7FF) >> 1] & 0xF;
        //g_u32_next2_op = g_zzw_slot_table[((g_s32_frame_no + 1) & 0x7FF)>>1]>>4;
		g_u32_next3_op = g_zzw_slot_table[u32_next_next_frm>>1] & 0xF;	
    }
    OAL_ASSERT((g_u32_next3_op <= ZZW_SLOT_INDEX_MAX), "");

    if(OAL_HOLD == g_zzw_rtc.u32_valid)
    {
        g_zzw_rtc.u32_valid = OAL_TRUE;
    }


    /*pwr config*/
    #ifdef KD001_RF8242
    #else
    rf_pwr_set(g_zzw_tx_pwr);
	#endif
    /*gain config*/
    if((0 == (s_u32_frm_int_cnt & 0x3ff)) && (g_zzw_node_status.state EQ ZZW_STATE_SCN))
    {
        g_u32_scan_gain_level = (g_u32_scan_gain_level + 1) % 3;
    }
    #ifdef MIMO2
    if(g_zzw_freq_set_flag)
    {
    	#if 0
        if(RFOP_2_TYPE(g_u32_next_op) EQ RX)
        {
        	g_u32_next_op = NOP;
        }
        //if(g_zzw_freq_set_flag EQ 2)
        //{
		//	g_zzw_freq_set_flag = OAL_FALSE;
        //}
        #else
        if(g_zzw_freq_set_flag EQ OAL_TRUE)
        {
        	if(RFOP_2_TYPE(g_u32_next_op) EQ RX)
        	{
        		g_u32_next_op = NOP;
        	}
        	else if(RFOP_2_TYPE(g_u32_next_op) EQ TX)
        	{
       			g_u32_next3_op = NOP;
   			}
       	}
        if(RFOP_2_TYPE(g_u32_current_op) EQ NOP)
        {
        	KS_LOG(0, 0XDDDB, g_zzw_freq_val);
			rf_sch_freq_set(g_zzw_freq_val, g_u16_freq_reg);
			g_zzw_freq_set_flag = OAL_FALSE;
        }
		else
		{
			g_zzw_freq_set_flag++;
		}
        #endif
    }
    #else
 	if(OAL_FALSE != g_zzw_freq_set_flag)
    {
        /* when g_zzw_freq_set_flag = 1,  g_u32_next_next_next_op set NOP. when this nop comes to the current op, excute freq set. */
        /* rf_sch_freq_set() should be excuted only once !! */
        if(g_zzw_freq_set_flag EQ 5)
        {
    	    #ifdef KD001_RF8242
    	    rf_sch_freq_set_8242(g_zzw_freq_val);
    	    #else
            rf_sch_freq_set(g_zzw_freq_val, g_u16_freq_reg);
            #endif
        }
        g_u32_next3_op = NOP; 
        g_zzw_freq_set_flag = (g_zzw_freq_set_flag + 1) & 0x7FF;        // protect time : 1024 frames
    }
	#endif
	if(g_s32_state_change > 0)
	{
		g_s32_state_change--;
        //g_u32_next_op = NOP;
		//g_u32_next2_op = NOP;
		g_u32_next3_op = NOP;

		//g_u32_tx_buf_in = 0;
		//g_u32_tx_buf_out = 0;
	}

	#if 0
	if((u32_next3_op_bak NEQ g_u32_next2_op) && (RFOP_2_TYPE(u32_next_next_op_bak) EQ TX))
	{
		#if 0
		if(TX EQ RFOP_2_TYPE(g_u32_next2_op))
		{
			//next is tx, but encode not ready, need drop it(for 1st power on, or slot table changed)
			g_u32_next2_op = NOP;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC106 , 1);    
		}
		if(ZZW_SLOT_SND_DF EQ (u32_next3_op_bak))
		{
			g_u32_tx_buf_in = (g_u32_tx_buf_in + 0x3 - 0x1) % 0x3;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC106 , 2);    			
		}
		#else
		g_u32_next2_op = u32_next3_op_bak;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC106 , 1);    
		#endif
	}
	
	if((u32_next_next_op_bak NEQ g_u32_next_op) && (RFOP_2_TYPE(u32_next_next_op_bak) EQ TX))
	{
		#if 0
		if(TX EQ RFOP_2_TYPE(g_u32_next_op))
		{
			//next is tx, but encode not ready, need drop it(for 1st power on, or slot table changed)
			g_u32_next_op = NOP;
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC105 , 1);    
		}
		if(TX EQ RFOP_2_TYPE(u32_next_next_op_bak))
		{
			//tx encode is doing , need cancel it
			g_u32_tx_encode_cancel = OAL_TRUE;
			if(ZZW_SLOT_SND_DF EQ (u32_next_next_op_bak))
			{
				g_u32_tx_buf_out = (g_u32_tx_buf_out + 0x3 - 0x1) % 0x3;
			}
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xC105 , 2); 			
		}
		#else
		g_u32_next_op = u32_next_next_op_bak;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC105 , 1);    
		#endif
	}
	#endif
    /*rtc负值表示扩长帧,正值表示缩短帧*/
    if(OAL_TRUE == g_zzw_rtc.u32_valid)
    {
        if((RFOP_2_TYPE(g_u32_next_op) EQ TX) && ((g_zzw_rtc.st_rtc.s32_rtc_value >= 550) || (g_zzw_rtc.st_rtc.s32_rtc_value <= -3506)))
        {
            //g_u32_next_op = NOP;
            g_zzw_rtc.u32_valid = OAL_HOLD;

        }
        else if(RFOP_2_TYPE(g_u32_next_op) EQ RX)
        {
            if((g_zzw_rtc.st_rtc.s32_frame_no & 0x7FF) || ((g_zzw_rtc.st_rtc.s32_rtc_value >= (SINT32)100) || (g_zzw_rtc.st_rtc.s32_rtc_value <= (SINT32)-100)))
            {
                g_u32_next_op = NOP;

				KS_LOG(OAL_TRACE_LEVEL_0 , 0xC104 , (UINT16)g_zzw_rtc.st_rtc.s32_frame_no);			
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xC104 , (UINT16)(g_zzw_rtc.st_rtc.s32_frame_no >> 16));			

        		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC104 , (UINT16)g_zzw_rtc.st_rtc.s32_rtc_value);			
				KS_LOG(OAL_TRACE_LEVEL_0 , 0xC104 , (UINT16)(g_zzw_rtc.st_rtc.s32_rtc_value >> 16));			
            }
            #if 0
            else
            {
                g_zzw_rtc.u32_valid = OAL_HOLD;
            }
            #endif
        }

        g_zzw_rtc.st_rtc.s32_frame_no = 0;
    }
    dfe_timing_update();

    if(RFOP_2_TYPE(g_u32_next2_op) EQ TX)
    {
        /*发送消息至ul_pro task，启动tx data encode*/
        stp_msg = ks_oal_msg_create(2);
        OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure");

        if(ZZW_SLOT_SND_SF == g_u32_next2_op)
        {
            stp_msg->msg_primitive_id = MSG_UL_BS_PROC_PRIMITIVE_ID;
        }
        else
        {
            stp_msg->msg_primitive_id = MSG_UL_DS_PROC_PRIMITIVE_ID;
            u32p_temp = (UINT32_PTR)&stp_msg->vp_msg_body;
            u32p_temp[0] = g_u32_tx_buf_out;
            g_u32_tx_buf_out = (g_u32_tx_buf_out + 1) % 0x3;
        }

        ks_oal_msg_send(stp_msg, OAL_SYS_TASK_ID + 3);
		g_u32_tx_enc_cnt++;

    }

    /*读取下下帧操作类型*/ // cparm
	if((ZZW_SLOT_SND_DF == g_u32_next3_op))
    {
	#if 1
        stp_msg = ks_oal_msg_create(2);
        OAL_ASSERT(NULL_PTR != stp_msg, "ks_oal_msg_create failure");
        stp_msg->msg_primitive_id = MSG_UL_DS_NEXT_TX_PRIMITIVE_ID;
        u32p_temp = (UINT32_PTR)&stp_msg->vp_msg_body;
	    u32p_temp[0] = g_u32_tx_buf_in;

        ks_oal_msg_send(stp_msg, OAL_SYS_TASK_ID + 3);
	    g_u32_tx_buf_in = (g_u32_tx_buf_in + 1) % 0x3;
	    KS_LOG(OAL_TRACE_LEVEL_0 , 0xC113 , (g_u32_tx_buf_out<<8) | g_u32_tx_buf_in);
	    if(ZZW_SLOT_SND_DF == g_u32_next2_op)
	    {
			OAL_ASSERT(((g_u32_tx_buf_in + 3 - g_u32_tx_buf_out) % 3 NEQ 0) , "");
		}
	#else
		trigger_cparm_pack_pkg(u32_pingpong_indication);
	#endif
    }

    /*freq config*/
    /*将Rx帧信息写入share ram，供xc4500 rx译码处理*/
    if((RX != RFOP_2_TYPE(g_u32_current_op)) && (RX == RFOP_2_TYPE(g_u32_next_op)))
    {
        stp_sarm_1643_4500->u32_rx_restart += 1; 
        stp_sarm_1643_4500->s16_rx_fn = (SINT16)g_s32_frame_no;
        stp_sarm_1643_4500->u8_node_id_local = (UINT8)g_zzw_node_status.lc_id;

		ks_dmac_ch_disable(KS_DMAC0_CP, DMA_CHAN_FOR_RX);
		ks_dmac_ch_disable(KS_DMAC0_CP, DMA_CHAN_FOR_RX2);

		ks_oal_delay(50);
		
		if((READ_REG32(0x17440018) & (1<<DMA_CHAN_FOR_RX)) != 0)
		{
			OAL_ASSERT(0, "dmac disable failure");
		}
		ks_dmac_set_llp(KS_DMAC0_CP, DMA_CHAN_FOR_RX, (UINT32)DL_RXBUFFER_START_ADDR0);
		ks_dmac_ch_start(KS_DMAC0_CP, DMA_CHAN_FOR_RX);	

		if((READ_REG32(0x17440018) & (1<<DMA_CHAN_FOR_RX2)) != 0)
		{
			OAL_ASSERT(0, "dmac disable failure");
		}
		ks_dmac_set_llp(KS_DMAC0_CP, DMA_CHAN_FOR_RX2, (UINT32)DL_RXBUFFER_START_ADDR1);
		ks_dmac_ch_start(KS_DMAC0_CP, DMA_CHAN_FOR_RX2);	

		if(OAL_TRUE == g_st_iq_dump.start_en)
		{		
			if(g_st_iq_dump.dma_cb_cnt EQ 0)
			{			
				ks_rxdfe_iq_dump_restart(0,g_st_iq_dump.dmac_ch_id);
				#if MIMO_2ANTS
				ks_rxdfe_iq_dump_restart(1,g_st_iq_dump.dmac_ch_id_ant2);
				#endif		
			}					
		}
		
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xD111 , (UINT16)g_u32_build_date);			
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xD111 , (UINT16)(g_u32_build_time >> 8));
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC103 , (UINT16)(((g_s32_frame_no) >> 16) & 0xFFFF));			

    }
    else
    {
		stp_sarm_1643_4500->u8_rx_state = (UINT8)g_zzw_node_status.state;
		if(OAL_TRUE == g_st_iq_dump.start_en)
		{
			if(OAL_FALSE == g_st_iq_dump.drop_rf_op_flag)	// only drop once
			{
				g_u32_next_op = NOP;
				ks_dmac_ch_disable(KS_DMAC0_CP, DMA_CHAN_FOR_IQ_DUMP);
				#if MIMO_2ANTS
				ks_dmac_ch_disable(KS_DMAC0_CP, DMA_CHAN_FOR_IQ_DUMP_ANT2);
				#endif				
				g_st_iq_dump.drop_rf_op_flag = OAL_TRUE;		
			}
		}
		u32_rf_reg_addr = READ_REG32(0x1724FFE4);
		if((u32_rf_reg_addr & 0xFFFF0000) EQ 0x12340000)
		{
			//low 16 bits of 0x1724FFE4 is rf reg addr
			KS_LOG(OAL_TRACE_LEVEL_0 , 0xD112 , (UINT16)u32_rf_reg_addr);
	    	rf_spi_read_reg(RF_SPI_USED, (UINT16)(u32_rf_reg_addr & 0xFFFF), &u32_rf_regs[0], KS_RF_CXPS8242);
	    	WRITE_REG32(0x1724FFE4, u32_rf_regs[0]);
	    }
    }

	KS_LOG(OAL_TRACE_LEVEL_0 , 0xC101 , (UINT16)((UINT16)(g_u32_next3_op << 12) | (g_u32_next2_op << 8) | (g_u32_next_op << 4) | (g_u32_current_op<<0)));    

    /*执行rf_sch event配置*/
    rf_sch_fsm[RFOP_2_TYPE(g_u32_current_op)][RFOP_2_TYPE(g_u32_next_op)](u32p_evt_ram_addr);

	#if 0
    if(TX EQ RFOP_2_TYPE(g_u32_next_op))
    {
    	g_u32_tx_evt_cnt++;
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC103 , (UINT16)g_u32_tx_enc_cnt);			
		KS_LOG(OAL_TRACE_LEVEL_0 , 0xC103 , (UINT16)g_u32_tx_evt_cnt);			
	    OAL_ASSERT(g_u32_tx_enc_cnt EQ g_u32_tx_evt_cnt , "");
    }
    #endif

    //ks_oal_mailbox_handle();
	#ifdef FUN_OTD
    if(g_s16_otd_flg >= 3)
    {
		g_s16_otd_flg = (g_s16_otd_flg + 1) & 0xFF;
		if(g_s16_otd_flg EQ 0)
		{
    		s_st_node_info_tbl.u8_otd_adj_flg = OAL_FALSE;
		}
	}
	#endif
    s_u32_frm_int_cnt++;
}

/********************************* END OF FILE ************************************************************************/
