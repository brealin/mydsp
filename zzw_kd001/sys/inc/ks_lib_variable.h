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
* 1.0 2019-06-17 created by FSLI
***********************************************************************************************************************/
#ifndef _KS_LIB_VARIABLE_H_
#define _KS_LIB_VARIABLE_H_
#include "ks_macro.h"
#include "ks_oal.h"
#include "ks_section.h"
#include "ks_kd001_main.h"

/* TCB of all the tasks */
extern UINT32 g_u32_oal_task_num;
extern UINT32 g_u32_oal_assert_info;
extern oal_task_tcb_t g_st_oal_task_tcb[OAL_TASK_AMT];

extern oal_scb_t g_st_oal_ipc_scb[OAL_IPC_SEMA_MAXNUM];
extern UINT16 g_u16_ipc_sema_index[OAL_IPC_SEMA_MAXNUM];
extern SINT32 oal_ipc_sema_maxnum;

#if defined(CORE_X1643)
extern VOLATILE_SINT32 cp1_x1643_def_flag;
extern OAL_TRACE_LOG_BUF_DESC_T* g_trace_log_buffer;
extern OAL_TRACE_STR_BUF_DESC_T* g_trace_str_buffer;
#else
extern VOLATILE_SINT32 cp1_xc4500_def_flag;
extern OAL_TRACE_LOG_BUF_DESC_T* g_trace_log_buffer;
extern OAL_TRACE_STR_BUF_DESC_T* g_trace_str_buffer;
#endif


#endif /* _KS_LIB_VARIABLE_H_ */
/**************************************************** END OF FILE *****************************************************/

