/******************************************************************************
 ** Module Name: <Taurus>.<cp_sys>.<framing>
 ** Author     : gjp
 ** Date       : 2020/5/21 @ 15:47:13
 ** Description: C header file of Module framing driver .
 ** Version    :
 **      First draft by xxx 20xx.xx.xx
 ******************************************************************************/

/******************************************************************************
 **                        Edit History                                       *
 ** ------------------------------------------------------------------------- *
 ** DATE             NAME                DESCRIPTION                               *
 ** 2020/02/27       GJP                   CREAT                    *
 **                                                                           *
 ******************************************************************************/

#ifndef _FRAMING_PROC_H_
#define _FRAMING_PROC_H_

VOID framing_config_init(UINT8 u8_frm_ch_id);
VOID framing_proc_config(UINT8 u8_frm_ch_id);
VOID framing_sym_num_config(UINT8 u8_frm_ch_id, UINT32 u32_sym_num, BOOLEAN b_is_pilot);

#endif
