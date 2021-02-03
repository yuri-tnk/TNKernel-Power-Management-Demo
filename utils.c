/**
*
*  Copyright (c) 2013,2016 Yuri Tiomkin
*  All Rights Reserved
*
*
*  Permission to use, copy, modify, and distribute this software in source
*  and binary forms and its documentation for any purpose and without fee
*  is hereby granted, provided that the above copyright notice appear
*  in all copies and that both that copyright notice and this permission
*  notice appear in supporting documentation.
*
*
*  THIS SOFTWARE IS PROVIDED BY YURI TIOMKIN "AS IS" AND ANY EXPRESSED OR
*  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
*  IN NO EVENT SHALL YURI TIOMKIN OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
*  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
*  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
*  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
*  THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "io430.h"

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>    // printf

#include "tn_config.h"
#include "tn.h"
#include "tn_utils.h"
#include "tn_port.h"
#include "prj_conf.h"
#include "types.h"
#include "uart.h"
#include "dt.h"
#include "func_prototypes.h"
#include "data_externals.h"


int vStrPrintf( char *outStr, int maxLen, const char *fmt, va_list args );

//----------------------------------------------------------------------------
void TestPrint(const char * fmt, ...)
{
 // Sem here protects 'g_test_print_buf' 

   tn_sem_acquire(&semTestPrint, TN_WAIT_INFINITE); // (RTOS_TICKS_PER_SECOND) * 2UL); // sleep until TX finished or stoped
// ????   memset(g_test_print_buf, 0, TESTPRINT_BUF_LEN);        //!< Clear the buffer
   g_test_print_buf[0] = 0;

   va_list  args;
   va_start( args, fmt);                          
   vStrPrintf(g_test_print_buf, TESTPRINT_BUF_LEN - 1, fmt, args); 

   uart1_tx_str(g_test_print_buf);                                   

   tn_sem_signal(&semTestPrint);
}

#if 0
//----------------------------------------------------------------------------
int msg_release_mem(SYS_MSG * msg)
{
   int rc = TERR_WPARAM;
   if(msg != NULL)
   {
      if(msg->mem_buf !=  NULL)
      {
        // rc = tn_dealloc(&g_sms_mem, (void *)msg->mem_buf);
      } 
   }

   return rc;
}
#endif

//----------------------------------------------------------------------------
BOOL is_pending_in_mailbox(TN_MAILBOX * mb, TN_TCB * task)
{
   CDLL_QUEUE * que;
   CDLL_QUEUE * que_next;
   TN_TCB * curr_task;
   int rc = FALSE;

   for(que = mb->inuse_wait_queue.next; 
           que != &mb->inuse_wait_queue; que = que_next)
   {
      que_next = que->next;

      curr_task = get_task_by_tsk_queue(que);

      if(curr_task != NULL && (curr_task == task))
      { 
         rc = TRUE;
         break;
      }
   }

   return rc;
}

//----------------------------------------------------------------------------
BOOL is_timers_lists_empty(void)
{
   BOOL rc = TRUE;
   int i;

   if(is_queue_empty(&tn_user_timer_list_gen) == TRUE)
   {
      for(i = 0; i < TN_USER_TICK_LISTS_CNT; i++)
      {
         if(is_queue_empty(&tn_user_timer_list_tick[i]) == FALSE)
         {
            rc = FALSE;
            break;
         }
      }
   }
   else
   {
      rc = FALSE;
   }

   return rc;
}

//----------------------------------------------------------------------------
int get_sem_cnt(TN_SEM * sem)
{
   TN_INTSAVE_DATA
   int rc = TERR_WPARAM;

   if(sem != NULL)
   {
      tn_disable_interrupt();

      if((sem->id_sem & TN_ID_MASK) != TN_ID_SEMAPHORE)
      {
         rc = TERR_NOEXS;
      }
      else
      {
         rc = (int)sem->count;
      }

      tn_enable_interrupt();
   }

   return rc;
}

//----------------------------------------------------------------------------
void wdt_restart(void)
{
   // stub -just for example
}

//----------------------------------------------------------------------------
void wdt_set_long(void)
{
  // stub -just for example
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


