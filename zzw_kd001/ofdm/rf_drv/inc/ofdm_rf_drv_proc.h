/******************************************************************************
 ** Module Name: <Taurus>.<cp_sys>.<rf_drv>
 ** Author     : Taurus DSP Team 
 ** Date       : 2020/06/10 @ 15:33:41
 ** Description: the register C header file of Module rf_ctrl .
 ** Version    : 
 **      First draft by xxx 20xx.xx.xx
 ******************************************************************************/
 
/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE        NAME                DESCRIPTION                               *
 ** 2020/6/10   rf_drv_proc.h          call rf ctrl  rf spi  rffe func to          *
 **                                cotrol sys   by gjp                        *
 **                                                                           *
 ******************************************************************************/

#ifndef _RF_DRV_PROC_H_
#define _RF_DRV_PROC_H_

#include "l1cc.h"

#ifdef __cplusplus
extern "C"
{
#endif

/***********************************************************************************************************************
* TYPE DEFINITION
***********************************************************************************************************************/
typedef VOID (*RF_SCH_FP)(UINT32_PTR);

/***********************************************************************************************************************
* MACRO DEFINITION
***********************************************************************************************************************/
#define FREQ_FILTER               (91625969) // 2 * 2^41/48000
#if 1
#if (defined CP1_X1643) || (defined CP1_XC4500)
#define DL_RXBUFFER_START_ADDR0    (0x17042800)
#define DL_RXBUFFER_START_ADDR1    (0x17043C00)
#else
#define DL_RXBUFFER_START_ADDR0    (0x17042800)
#define DL_RXBUFFER_START_ADDR1    (0x17043C00)
#endif
#endif
#define AGC_TARGET_RSSI_IN_DB     (0x46)
#define RF_SCH_SPI_ENABLE         (1)

#define OAL_HOLD                  ((unsigned short int)2)

#define NOP      0
#define RX       1
#define TX       2

typedef struct _DFE_TIMING_INFO_T_
{
    UINT32 u32_timing_update_valid;
    UINT32 u32_timing_update_frame_type;
    SINT32 s32_timing_update_value;
    SINT32 s32_timing_rst_value;
    UINT32 u32_frame_rst_flag;
}DFE_TIMING_INFO_T;

typedef enum _eKS_CX9261_LO_FREQ_RANGE_T
{
    CX9261_LO_FREQ_RANGE_ERROR                   = -1,
    CX9261_LO_FREQ_RANGE_2700MHZ_TO_1900MHz      = 0,
    CX9261_LO_FREQ_RANGE_1900MHZ_TO_1350MHz      = 1,
    CX9261_LO_FREQ_RANGE_1350MHZ_TO_950MHz       = 2,
    CX9261_LO_FREQ_RANGE_950MHZ_TO_675MHz        = 3,
    CX9261_LO_FREQ_RANGE_675MHZ_TO_475MHz        = 4,
    CX9261_LO_FREQ_RANGE_475MHZ_TO_337p5MHz      = 5,
    CX9261_LO_FREQ_RANGE_337p5MHZ_TO_237p5MHz    = 6,
    CX9261_LO_FREQ_RANGE_237p5MHZ_TO_168p75MHz   = 7,
    CX9261_LO_FREQ_RANGE_168p75MHZ_TO_118p75MHz  = 8,
    CX9261_LO_FREQ_RANGE_118p75MHZ_TO_84p375MHz  = 9,
    CX9261_LO_FREQ_RANGE_84p375MHZ_TO_59p375MHz  = 10,
    CX9261_LO_FREQ_RANGE_59p375MHZ_TO_42p1875MHz = 11,
}eKS_CX9261_LO_FREQ_RANGE;
/***********************************************************************************************************************
* FUNCTION DECLARATION
***********************************************************************************************************************/
VOID rf_drv_init();
VOID rf_drv_deinit();
VOID rf_sch_evt_int();
VOID rf_sch_frm_int();
VOID rf_sch_rx2rx();
VOID rf_sch_rx2tx();
VOID rf_sch_rx2nop();
VOID rf_sch_tx2rx();
VOID rf_sch_tx2tx();
VOID rf_sch_tx2nop();
VOID rf_sch_nop2rx();
VOID rf_sch_nop2tx();
VOID rf_sch_nop2nop();
VOID rf_sch_gain_set(SINT32 s32_rx_gain);

#ifdef __cplusplus
}
#endif

/**---------------------------------------------------------------------------*/

#endif // _RF_DRV_REG_H_
