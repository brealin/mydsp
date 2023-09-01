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
* 1.0 2021-12-27 created
***********************************************************************************************************************/

#ifndef _KS_KD001_MAIN_H_
#define _KS_KD001_MAIN_H_

#include "ks_oal.h"

#if defined(CORE_X1643)
#define OAL_SYS_TASK_ID                             (((CORE_ID_X1643)<<7) | 0x0)    
#define OAL_TASK_AMT                                (5)    
#else	// #elif defined(CORE_XC4500)
#define OAL_SYS_TASK_ID                             (((CORE_ID_XC4500)<<7) | 0x0)  /* 0x80 */
#define OAL_TASK_AMT                                (3)       
#endif

#define OAL_HEAP_AMT                                (3)
#define OAL_STACK_AMT                               (3)

#define OAL_STACK_DTCM_LEVEL                        (0x0) /* DTCM Stack area ID */
#define OAL_STACK_SRAM_LEVEL                        (0x1) /* SHRAM cacheable  Stack area ID */
#define OAL_STACK_DRAM_LEVEL                        (0x2) /* DRAM non_cacheable  Stack area ID */

#define OAL_STACK_DTCM_ID                           ((UINT32)OAL_STACK_DTCM_LEVEL << 24)
#define OAL_STACK_SRAM_ID                           ((UINT32)OAL_STACK_SRAM_LEVEL << 24)
#define OAL_STACK_DRAM_ID                           ((UINT32)OAL_STACK_DRAM_LEVEL << 24)

#define OAL_STACK_DTCM_SIZE                         (UINT32)(0x1020)  /* unit: 16 bits */
#define OAL_STACK_SRAM_SIZE                         (UINT32)(0x1000)  /* unit: 16 bits */
#define OAL_STACK_DRAM_SIZE                         (UINT32)(0x1000)  /* unit: 16 bits */

#if OAL_MSG_QUEUE
#define OAL_HEAP_DTCM_SIZE                          (0x800) /* 8KB,UNIT: DWORD */
#else
#define OAL_HEAP_DTCM_SIZE                          (0x1400) /* 20KB,UNIT: DWORD */
#endif
#define OAL_HEAP_SRAM_SIZE                          (0x400)  /* temp:0x400;real:0x4000, UNIT: DWORD */
#define OAL_HEAP_DRAM_SIZE                          (0x200)  /* temp:0x200;real: 0x10C000,long duration,UNIT: DWORD */

#define ARM2DSP_DL_DATA_BUFSIZE                     (8192)
#define ARM2DSP_DL_DATA_DST_TASKID                  (10)
#define MSG_UL_BS_PROC_PRIMITIVE_ID                 (0)
#define MSG_UL_DS_PROC_PRIMITIVE_ID                 (1)
#define MSG_UL_MODULATION_PRIMITIVE_ID              (1)
#define MSG_UL_DFT_DATA_PRIMITIVE_ID                (2)
#define MSG_DL_RTC_REPORT_PRIMITIVE_ID              (3)
#define MSG_UL_DS_NEXT_TX_PRIMITIVE_ID              (4)
#define MSG_DL_RSSI_REPORT_PRIMITIVE_ID             (5)
#define L1CC_UL_PROC_MAIN_TASK_ID                   (((CORE_ID_X1643)<<7) | 0x3)
#define L1CC_UL_MOD_MAIN_TASK_ID                    (((CORE_ID_X1643)<<7) | 0x5)

#endif
