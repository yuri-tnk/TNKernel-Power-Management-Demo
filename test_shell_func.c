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

#include "tn_config.h"
#include "tn.h"
#include "tn_port.h"
#include "prj_conf.h"
#include "types.h"
#include "uart.h"
#include "dt.h"
#include "func_prototypes.h"
#include "data_externals.h"



int sh_exec_cmd_psw(SHELLINFO * sh_info);
int sh_exec_cmd_ver(SHELLINFO * sh_info);
int sh_exec_cmd_1(SHELLINFO * sh_info);
int sh_exec_dlt(SHELLINFO * sh_info);
int sh_exec_cmd_3(SHELLINFO * sh_info);
int sh_exec_dump_stk(SHELLINFO * sh_info);
int sh_exec_start_job2(SHELLINFO * sh_info);
int sh_exec_rs485(SHELLINFO * sh_info);

const SHELLCMD g_shell_cmd_arr[] =
{
   {"psw",            sh_exec_cmd_psw},
   {"ver",            sh_exec_cmd_ver},
   {"cmd1",           sh_exec_cmd_1},
   {"dlt",            sh_exec_dlt},
   {"time",           sh_exec_cmd_3},
   {"dump_stk",       sh_exec_dump_stk},
   {"job2",           sh_exec_start_job2},
   {"rs485",          sh_exec_rs485}
};

const int g_shell_cmd_arr_size = sizeof(g_shell_cmd_arr)/sizeof(SHELLCMD);

//----------------------------------------------------------------------------
int sh_exec_cmd_psw(SHELLINFO * sh_info)
{
   TestPrint("This is a PSW cmd.\r\n");
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_cmd_ver(SHELLINFO * sh_info)
{
   TestPrint("Ver - 3.0\r\n");
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_cmd_1(SHELLINFO * sh_info)
{
   TestPrint("This is a CMD1.\r\n");
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_dlt(SHELLINFO * sh_info)
{
   LTTMRENTRY * parr;
   LTT_DT exec_dt;     
   LTT_DT base_dt;
   int i;   
   int rc;
   unsigned int flags;

   parr = ltt_timers_get_arr();

   rc = tn_sem_acquire(&semTimerLT, TOUT_200_MSEC);
   if(rc == TERR_NO_ERR)
   {
     
      for(i = 0; i < MAX_LT_TIMERS; i++)
      {
         if((parr->flags & LTTL_IS_FREE) != LTTL_IS_FREE)
         {
            memset(&base_dt, 0, sizeof(base_dt));
            pk_to_dt(&parr->base_dt,
                     &base_dt);
            memset(&exec_dt, 0, sizeof(exec_dt));
            pk_to_dt(&parr->exec_dt,
                    &exec_dt);

            TestPrint("\r\ntimer_id: %ld timeout: %ld ", 
                         parr->timer_id, parr->timeout);

            flags = parr->flags;
            if((flags & LTTL_EXEC_NOW) == LTTL_EXEC_NOW)
            {   
               TestPrint("LTTL_EXEC_NOW ");
            }
            if((flags & LTT_ONESHOT) == LTT_ONESHOT)
            { 
               TestPrint("LTT_ONESHOT ");
            } 
            if((flags & LTT_RELATIVE_MODE) == LTT_RELATIVE_MODE) //-- relative mode
            {
               TestPrint("LTT_RELATIVE_MODE ");
            }    

            TestPrint("\r\n");
            TestPrint("base_dt: %d-%d-%d %02d:%02d:%02d\r\n", 
                     base_dt.tm_year,
                     base_dt.tm_mon,
                     base_dt.tm_mday,
                     base_dt.tm_hour,
                     base_dt.tm_min, 
                     base_dt.tm_sec);
            TestPrint("exec_dt: %d-%d-%d %02d:%02d:%02d\r\n", 
                     exec_dt.tm_year,
                     exec_dt.tm_mon,
                     exec_dt.tm_mday,
                     exec_dt.tm_hour,
                     exec_dt.tm_min, 
                     exec_dt.tm_sec);

            TestPrint("func: 0x%08lX data: 0x%08lX\r\n",
                     (unsigned long)(parr->callback_func),
                     (unsigned long)(parr->callback_func_data));
         }
         parr++;
      }

      (void)tn_sem_signal(&semTimerLT);
   }
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_cmd_3(SHELLINFO * sh_info)
{
   LTT_DT dt;
   const char *  g_dw[7]   = {"Sun", "Mon", "Tue", "Wen", "Thu", "Fri", "Sat"};
   const char *  g_mon[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", 
                              "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

   rtc_get_clock(&dt);            //!< [OUT]

   TestPrint("\r\nTime: %d %s %d DW: %d(%s) %02d:%02d:%02d\r\n",
             dt.tm_year,
             (char*)g_mon[dt.tm_mon - 1],
             dt.tm_mday,
             dt.tm_wday,
             (char*)g_dw[(int)dt.tm_wday],
             dt.tm_hour,
             dt.tm_min,
             dt.tm_sec);
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_dump_stk(SHELLINFO * sh_info)
{
   CDLL_QUEUE * que;
   CDLL_QUEUE * que_next;
   TN_TCB * task;
   unsigned int * ptr;
   unsigned long  end_stack_ul;

   TestPrint("The stack related values are in the \'int\'(not bytes)\r\n");

   for(que = tn_create_queue.next; que != &tn_create_queue; que = que_next)
   {
      que_next = que->next;

      task = get_task_by_create_queue(que);
      end_stack_ul = (unsigned long)task->stk_start - (unsigned long)(task->stk_size << 1) + sizeof(int); // Stack End - Minimal Addr
      ptr = (unsigned int *)end_stack_ul;
      for(;;)
      {
         if(*ptr != TN_FILL_STACK_VAL)
            break;
         ptr++;
      }

      TestPrint("Task: %2d  Usage: %4d Size: %4d\r\n",
                task->base_priority,
                (int)(task->stk_start - ptr + 1), // case ptr was ++
                task->stk_size);
   }
   
   return 0;
}

//----------------------------------------------------------------------------
int sh_exec_start_job2(SHELLINFO * sh_info)
{
   int rc = TERR_NO_ERR;
   SYS_MSG msg= {0};

   msg.op_code = SYS_MSG_LTT_1_EVENT;

   (void)tn_mailbox_send(&mailboxSysOp,
                        &msg,
                        sizeof(SYS_MSG),
                        TN_NO_WAIT); 

   return rc;
}

//----------------------------------------------------------------------------
int sh_exec_rs485(SHELLINFO * sh_info)
{
   int rc = TERR_NO_ERR;
   unsigned char * cmd_arg = NULL;
    UARTDS * p_uart = &g_p_uart_ds->g_uart1_ds;

   cmd_arg = sh_info->argv[1];
   if(cmd_arg == NULL)
   {
      TestPrint("Syntax: rs485 off\r\n");
   }
   else
   {
      if(strcasecmp((char*)cmd_arg, "off")  == 0)
      {
         TestPrint("rs485 will be closed.\r\n");
         (void)tn_task_sleep(TOUT_1_SEC);

         bsp_uart_close(p_uart);
      }
      else
      {
         TestPrint("Syntax: rs485 off\r\n");
         rc = -1; 
      }
   }
   return rc;

}  

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
