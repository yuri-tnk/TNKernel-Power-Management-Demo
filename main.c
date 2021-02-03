/*

  Copyright © 2004, 2016 Yuri Tiomkin
  All rights reserved.

  Permission to use, copy, modify, and distribute this software in source
  and binary forms and its documentation for any purpose and without fee
  is hereby granted, provided that the above copyright notice appear
  in all copies and that both that copyright notice and this permission
  notice appear in supporting documentation.

  THIS SOFTWARE IS PROVIDED BY THE YURI TIOMKIN AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL YURI TIOMKIN OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
  OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
  OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
  SUCH DAMAGE.

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


/*
   Sleep: each OS ticks, when all tasks finished their current job, CPU stopped.
          Wakeup - by any hardware interrupt, including OS ticks

   Deep sleep:  CPU stopped, OS ticks and RTC 1 sec updates are disabled, the system is set
                for the minimal power consumption(some sub system are disabled)
                Wakeup - by the user enabled hardware interrups (includes
                RTC alarm for the long term timers)      

   Operations - buttons 
                     - Button 1 - when pressed and hold - LED_ID_1 blinks 
                     - Button 2 (by interrupt when system is in the deep sleep)
                                - wakeup from deep sleep, reopen UART and returns 
                                  to the regular (no deep sleep) operating 
              - ltt timer - do 'job2' (LED blink 2 times total - 0.3 sec , 
                                       sleep 3 sec,  LED blink 2 times total)  
                            then - go to deep-sleep again

   Debug terminal: by RS485     
                   when RS485 is on, no deep sleep
                   may be closed by 'rs485 off' cmd (the system goes to deep sleep) 

   Wakeup from the deep sleep - Button 2  - (permanent)
                              - ltt timer - (each 2 min) - just for the 'job2', 
                                                           then go to deep sleep again

*/


//----------- Tasks ----------------------------------------------------------

//-- The OS ticks task - must

#define  TASK_OS_TICK_PRIORITY       0
#define OS_TICK_TASK_STACK_SIZE    128
TN_ALIGN_ATTR TN_TCB tn_os_tick_task;
TN_ALIGN_ATTR_START unsigned int tn_os_tick_task_stack[OS_TICK_TASK_STACK_SIZE] TN_ALIGN_ATTR_END;
void tn_os_tick_task_func(void * param);

//------------------------------------------

#if defined  USE_DYN_OBJ

#define SYS_OBJ_MEM_SIZE  4096
#pragma data_alignment = 4
__no_init unsigned char sys_obj_mem_arr[SYS_OBJ_MEM_SIZE];

#endif

#define  TASK_USER_HIPRI_PRIORITY    1
#define  TASK_UART1_RX_PRIORITY      3
#define  TASK_SYSOP_PRIORITY         6
#define  TASK_SHELL_PRIORITY         7
#define  TASK_IO_PRIORITY            8

#define  TASK_USER_HIPRI_STK_SIZE  512
#define  TASK_UART1_RX_STK_SIZE    512
#define  TASK_SYSOP_STK_SIZE      1024
#define  TASK_SHELL_STK_SIZE      1024
#define  TASK_IO_STK_SIZE          512

__no_init unsigned int task_user_hipri_stack[TASK_USER_HIPRI_STK_SIZE];
__no_init unsigned int task_uart1_rx_stack[TASK_UART1_RX_STK_SIZE];
__no_init unsigned int task_SysOp_stack[TASK_SYSOP_STK_SIZE];
__no_init unsigned int task_shell_stack[TASK_SHELL_STK_SIZE];
__no_init unsigned int task_io_stack[TASK_IO_STK_SIZE];

__no_init TN_TCB  task_user_hipri;
__no_init TN_TCB  task_uart1_rx;
__no_init TN_TCB  task_SysOp;
__no_init TN_TCB  task_shell;
__no_init TN_TCB  task_io;

void task_user_hipri_func(void * par);
void task_uart1_rx_func(void * par);
void task_SysOp_func(void * par);
void task_shell_func(void * par);
void task_io_func(void * par);

void idle_hook_func(void * par);

//-------- Semaphores -----------------------

__no_init TN_SEM semTxUart1;             //!< UART1 semaphores 
__no_init TN_SEM semTxBufUart1;
__no_init TN_SEM semTxUart2;             //!< UART2 semaphores 
__no_init TN_SEM semTxBufUart2;
__no_init TN_SEM semTestPrint;  // for TestPrint() func
__no_init TN_SEM semTimerLT;
__no_init TN_SEM semRTC;
__no_init TN_SEM semCPUFreq;

//-------- Queues -----------------------

#define QUEUE_ACTIONS_SIZE           8
#define QUEUE_ACTIONS_MEM_BUF_SIZE  (DQUEUE_ENTRY_SIZE * QUEUE_ACTIONS_SIZE)

TN_DQUEUE queueShellEvents;
#pragma data_alignment=4
unsigned char queueShellEventsMem[QUEUE_ACTIONS_MEM_BUF_SIZE];

//-------- Mailboxes -----------------------

__no_init TN_MAILBOX mailboxSysOp;

#define  MAILBOX_SYSOP_NUM_ELEM       8UL
#define  MAILBOX_SYSOP_ELEM_SIZE      sizeof(SYS_MSG)
#define  MAILBOX_ENTRY1_TOTAL_SIZE    MAILBOX_ENTRY_SIZE(MAILBOX_SYSOP_ELEM_SIZE)
#define  MAILBOX_SYSOP_DATABUF_SIZE   (MAILBOX_ENTRY1_TOTAL_SIZE * MAILBOX_SYSOP_NUM_ELEM)

TN_ALIGN_ATTR_START unsigned char mailboxSysOpBuf[MAILBOX_SYSOP_DATABUF_SIZE]  TN_ALIGN_ATTR_END;

__no_init TN_MAILBOX mailboxUserHiPri;

#define  MAILBOX_USER_HIPRI_NUM_ELEM      8UL
#define  MAILBOX_USER_HIPRI_ELEM_SIZE     sizeof(SYS_MSG)
#define  MAILBOX_ENTRY2_TOTAL_SIZE        MAILBOX_ENTRY_SIZE(MAILBOX_USER_HIPRI_ELEM_SIZE)
#define  MAILBOX_USER_HIPRI_DATABUF_SIZE  (MAILBOX_ENTRY2_TOTAL_SIZE * MAILBOX_USER_HIPRI_NUM_ELEM)

TN_ALIGN_ATTR_START unsigned char mailboxUserHiPriBuf[MAILBOX_USER_HIPRI_DATABUF_SIZE]  TN_ALIGN_ATTR_END;

//-------- Timers -----

__no_init TN_TIMER tmrPreventDS;
__no_init TN_TIMER tmrJob2;

volatile long g_llt1_timer;

//-------- Misc -------

extern idle_task_hook_func  tn_idle_task_hook_func;

   //--- Non-OS globals --

__no_init char g_test_print_buf[TESTPRINT_BUF_LEN];

__no_init BSP_UART_DS  g_uart_ds;
BSP_UART_DS * g_p_uart_ds = &g_uart_ds;     //!< global UART data

__no_init BSP_SHELL_DS  g_shell_ds;
BSP_SHELL_DS * g_p_shell_ds = &g_shell_ds;

RTCDATA g_rtc_data;
RTCDATA * g_prtc = &g_rtc_data;

DEEPSLEEPDATA g_deep_sleep_data;

LTTMRENTRY g_ltt_timers_arr[MAX_LT_TIMERS];

__no_init LEDSDATA g_leds_arr[NUM_LEDS]; 
__no_init BTNDATA  g_btn_arr[NUM_BUTTONS];

static const LTT_DT g_start_dt = 
{
   .tm_sec  = 0,
   .tm_min  = 0,
   .tm_hour = 1,
   .tm_mday = 1,
   .tm_mon  = 1,
   .tm_year = 2016,
   .tm_wday = 0
};

//--- Timer callback functions

void timer_job2_func(TN_TIMER * tmr, void * param);
int ltt1_timer_handler(int timer_id, LTT_DT_PK lt_timestamp, void * data);

//--- Local file functions prototypes

static void start_job2(void);
static void stop_job2(void);

//----------------------------------------------------------------------------
int tn_app_init(void)
{
   int rc;

   tn_idle_task_hook_func = idle_hook_func;

   //--- OS ticks task - MUST. 
   //--  The creation option for this task must be TN_TASK_OS_TICK

   tn_os_tick_task.id_task = 0UL;
   rc = tn_task_create(&tn_os_tick_task,           //-- task TCB
                        tn_os_tick_task_func,      //-- task function
                        0,                         //-- task priority
                        &(tn_os_tick_task_stack    //-- task stack first addr in memory
                        [OS_TICK_TASK_STACK_SIZE - 1]),
                        OS_TICK_TASK_STACK_SIZE,   //-- task stack size (in int,not bytes)
                        NULL,                      //-- task function parameter
                        TN_TASK_OS_TICK);          //-- Creation option
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

  //--- Task Hi Prio

   task_user_hipri.id_task = 0UL;
   rc = tn_task_create(&task_user_hipri,           //-- task TCB
                 task_user_hipri_func,             //-- task function
                 TASK_USER_HIPRI_PRIORITY,         //-- task priority
                 &(task_user_hipri_stack           //-- task stack first addr in memory
                    [TASK_USER_HIPRI_STK_SIZE-1]),
                 TASK_USER_HIPRI_STK_SIZE,         //-- task stack size (in int,not bytes)
                 NULL,                             //-- task function parameter
                 TN_TASK_START_ON_CREATION);       //-- Creation option
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

  //--- Task Rx UART1

   task_uart1_rx.id_task = 0UL;
   rc = tn_task_create(&task_uart1_rx,             //-- task TCB
                 task_uart1_rx_func,               //-- task function
                 TASK_UART1_RX_PRIORITY,           //-- task priority
                 &(task_uart1_rx_stack             //-- task stack first addr in memory
                    [TASK_UART1_RX_STK_SIZE-1]),
                 TASK_UART1_RX_STK_SIZE,           //-- task stack size (in int,not bytes)
                 NULL,                             //-- task function parameter
                 TN_TASK_START_ON_CREATION);       //-- Creation option
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   //--- Task SysOp

   task_SysOp.id_task = 0;
   rc = tn_task_create(&task_SysOp,                 //-- task TCB
                       task_SysOp_func,             //-- task function
                       TASK_SYSOP_PRIORITY,         //-- task priority
                       &(task_SysOp_stack           //-- task stack first addr in memory
                          [TASK_SYSOP_STK_SIZE-1]),
                       TASK_SYSOP_STK_SIZE,         //-- task stack size (in int,not bytes)
                       NULL,                        //-- task function parameter
                       TN_TASK_START_ON_CREATION);  //-- Creation option
   if(rc != TERR_NO_ERR)
   { 
      goto err_exit;
   }

   //--- Task Shell

   task_shell.id_task = 0;
   rc = tn_task_create(&task_shell,                 //-- task TCB
                 task_shell_func,                   //-- task function
                 TASK_SHELL_PRIORITY,               //-- task priority
                 &(task_shell_stack                 //-- task stack first addr in memory
                    [TASK_SHELL_STK_SIZE-1]),
                 TASK_SHELL_STK_SIZE,               //-- task stack size (in int,not bytes)
                 NULL,                              //-- task function parameter
                 TN_TASK_START_ON_CREATION);        //-- Creation option
   if(rc != TERR_NO_ERR)
   { 
      goto err_exit;
   }
   //--- Task IO

   task_io.id_task = 0;
   rc = tn_task_create(&task_io,                    //-- task TCB
                 task_io_func,                      //-- task function
                 TASK_IO_PRIORITY,                  //-- task priority
                 &(task_io_stack                    //-- task stack first addr in memory
                    [TASK_IO_STK_SIZE-1]),
                 TASK_IO_STK_SIZE,                  //-- task stack size (in int,not bytes)
                 NULL,                              //-- task function parameter
                 TN_TASK_START_ON_CREATION);        //-- Creation option
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   //--- Semaphores
   semTxUart1.id_sem = 0;
   rc = tn_sem_create(&semTxUart1,
                      0,      //-- Start value
                      1);     //-- Max value
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   semTxBufUart1.id_sem = 0;
   rc = tn_sem_create(&semTxBufUart1,
                      1,      //-- Start value
                      1);     //-- Max value
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }
   //------------
   semTxUart2.id_sem = 0;
   rc = tn_sem_create(&semTxUart2,
                      0,      //-- Start value
                      1);     //-- Max value
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   semTxBufUart2.id_sem = 0;
   rc = tn_sem_create(&semTxBufUart2,
                      1,      //-- Start value
                      1);     //-- Max value
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

//-----------------------------
   semTestPrint.id_sem = 0UL;
   rc = tn_sem_create(&semTestPrint,
                      1,  //-- Start value
                      1);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   semTimerLT.id_sem = 0UL;
   rc = tn_sem_create(&semTimerLT,
                      1,  //-- Start value
                      1);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   semRTC.id_sem = 0UL;
   rc = tn_sem_create(&semRTC,
                      1,  //-- Start value
                      1);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   semCPUFreq.id_sem = 0UL;
   rc = tn_sem_create(&semCPUFreq,
                      1,  //-- Start value
                      1);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   //--- Queues

   queueShellEvents.id_dqueue = 0UL;
   rc = tn_dqueue_create(&queueShellEvents,
                         QUEUE_ACTIONS_SIZE,
                         &queueShellEventsMem[0],  //-- Ptr to already existing array of void * to store data queue entries.Can be NULL
                         QUEUE_ACTIONS_MEM_BUF_SIZE);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   //--- Mailboxes

   mailboxSysOp.id_mailbox = 0UL;
   rc =  tn_mailbox_create(&mailboxSysOp, 
                          MAILBOX_SYSOP_NUM_ELEM,  //long num_elem,
                          MAILBOX_SYSOP_ELEM_SIZE, //elem_size,  
                          &mailboxSysOpBuf[0],
                          MAILBOX_SYSOP_DATABUF_SIZE);   
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   mailboxUserHiPri.id_mailbox = 0UL;
   rc =  tn_mailbox_create(&mailboxUserHiPri, 
                          MAILBOX_USER_HIPRI_NUM_ELEM,  //long num_elem,
                          MAILBOX_USER_HIPRI_ELEM_SIZE, //elem_size,  
                          &mailboxUserHiPriBuf[0],
                          MAILBOX_USER_HIPRI_DATABUF_SIZE);   
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   //--- Timers

   tmrPreventDS.id_timer = 0UL;
   rc = tn_timer_create(&tmrPreventDS,
                        timer_dummy_func,
                        NULL);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

   tmrJob2.id_timer = 0UL;
   rc = tn_timer_create(&tmrJob2,
                        timer_job2_func,
                        NULL);
   if(rc != TERR_NO_ERR)
   {
      goto err_exit;
   }

err_exit:
    
   return rc;
}

//----------------------------------------------------------------------------
int main(void)
{
   int rc;

   WDTCTL = WDTPW + (unsigned short)WDTHOLD;

   memset(&g_deep_sleep_data, 0, sizeof(DEEPSLEEPDATA));

   sys_init_cpu();

   sh_init(g_p_shell_ds);

   //-- Powered the system from the internal  battery
   P7OUT &= ~BIT6; // Close Vext(sanity)
   P7OUT |= BIT5;  // Open switch to Vbat         
   //-----------------------------------------------

   (void)memset(g_p_uart_ds, 0, sizeof(BSP_UART_DS));

   memset(g_prtc, 0, sizeof(RTCDATA));

   leds_init();
   buttons_init();
   ltt_timers_init();

   rc = rtc_init(g_prtc, (LTT_DT *)&g_start_dt);  
   if(rc != TERR_NO_ERR)
   {
      for(;;)
      {
      }
   }


   //-- if no err, should never returns
#if defined  USE_DYN_OBJ
 
   rc = tn_start_system(sys_obj_mem_arr,
                        (unsigned long) SYS_OBJ_MEM_SIZE,
                        NULL); // NULL is OK for the MSP430X  (int_stack_mem);
#else

   rc = tn_start_system(NULL,
                        0UL,
                        NULL); // NULL is OK for the MSP430X  (int_stack_mem);
#endif

   // if we are here, something goes wrong

   if(rc != TERR_NO_ERR)
   {
      // Add your own error handler here, (if you are really need it)
      for(;;)
      {
      }
   }
   return 0;
}

//----------------------------------------------------------------------------
// Must
//----------------------------------------------------------------------------
void tn_os_tick_task_func(void * param) 
{
   int rc;

   for(;;)
   {
      rc = tn_sem_acquire(&tn_sys_tick_sem, TN_WAIT_INFINITE);
      if(rc >= 0) // OK
      {
         (void)tn_os_timers_tick_proc();
#if defined USER_TIMERS
         (void)tn_user_timers_tick_proc();
#endif
       //--- The project related each OS tick processing 

         button_processing();
         sys_led_processing();

       //---------------------
      }
   }
}

//----------------------------------------------------------------------------
void task_uart1_rx_func(void * par)
{
#define DRV_UART1_RX_BUF_SIZE 32  
 
   char rx_buf[DRV_UART1_RX_BUF_SIZE];
   int i;
   int nbytes;

   UARTDS * p_uart = &g_p_uart_ds->g_uart1_ds;
 
   (void)bsp_uart_open(p_uart,
                       NULL, // const char* name,
                       0,    // int32_t flags,
                       (long)CFG_UART1_38400_8N1_20MHZ);
   for(;;)
   {
      // blocking reading
      nbytes = bsp_uart_read(p_uart, 
                             &rx_buf[0], 
                             DRV_UART1_RX_BUF_SIZE);    
      if(nbytes > 0)
      {
         for(i = 0; i < nbytes; i++)
         {
            sh_input(&g_p_shell_ds->shell_info,
                     rx_buf[i]);
         }
      }
      else
      {
         if(nbytes == -ENXIO) 
         {  // UART was closed - there is no tn_task_sleep() inside bsp_uart_read() now
         
            (void)tn_task_sleep(10);
         }
         else
         {
            // To Do
         }
      } 
   }
}

//----------------------------------------------------------------------------
void task_user_hipri_func(void * par)
{
   SYS_MSG msg;
   static int fDoAlarm = FALSE;
   long res;
   int rc;
  
//---- Long term timers - have to be registered when OS is running

   g_llt1_timer = ltt_register_timer(LTT_RELATIVE_MODE, // no oneshot - it is periodic timer
                                     NULL, // reserved, may be NULL
                                     2UL,           //timeout - in minutes
                                     NULL, 
                                     ltt1_timer_handler);

   if(g_llt1_timer < 1L) 
   {
      for(;;)
      {
      }
   }

   for(;;)
   {
      res = tn_mailbox_receive(&mailboxUserHiPri,
                               &msg,                 // [OUT]
                               sizeof(SYS_MSG),      // unsigned long max_len,
                               TN_WAIT_INFINITE);    //  unsigned long timeout
      if(res > 0L) // OK
      {
         switch(msg.op_code)
         {
            case SYS_MSG_INT_BTN0:  

               switch_to_op_mode(SYS_MSG_INT_BTN0); //!< may come as a deep sleep wake-up interrupt

               msg.op_code = SYS_MSG_WAKEUP_BTN0;

               rc = tn_mailbox_send(&mailboxSysOp,
                                    &msg,
                                    sizeof(SYS_MSG),
                                    TN_NO_WAIT); 
               break;

            case SYS_MSG_RTC_ALARM:

               switch_to_op_mode(SYS_MSG_RTC_ALARM); //!< may come as deep sleep wake-up interrupt

               fDoAlarm = TRUE;  

               break;

            case SYS_MSG_RTC_1SEC: // Never comes in the deep sleep mode

               //--- Update RTC - it is a safe time to do it

               rtc_update_ptr();

               if(fDoAlarm == TRUE) // aligned to 1 sec action
               { 
                  fDoAlarm = FALSE;

                  rc = ltt_alarm_proc(NULL);
                  if(rc < 0) // FatalError
                  {
                    // ToDo  
                  } 
               }

               break;

            default:

               break; 
         }
      }  
   }
}

//----------------------------------------------------------------------------
void task_shell_func(void * par)
{
   int rc;
   unsigned long rx_data;
 
   for(;;)
   {
      rc = tn_dqueue_receive(&queueShellEvents, (TN_DQUEUE_ELEMENT *)&rx_data, TN_WAIT_INFINITE);
      if(rc >= 0) // OK
      {
         if(rx_data == (unsigned long)EVT_EXEC_SHELL)
         {
            (void)sh_do_cmd_exec(&g_p_shell_ds->shell_info);
         }  
         else
         {
            TestPrint("Unknown msg.\r\n");
         }
      }
   }
}

//----------------------------------------------------------------------------
void task_SysOp_func(void * par)
{
   SYS_MSG msg;
   long res;
   int rc;
   UARTDS * p_uart = &g_p_uart_ds->g_uart1_ds;

#if 0
   set_led_on(LED_ID_1);
#endif

   for(;;)
   {
      res = tn_mailbox_receive(&mailboxSysOp,
                               &msg,        
                               sizeof(SYS_MSG),
                               TOUT_5_SEC); 
      if(res > 0L) // OK
      {
         switch(msg.op_code)
         {
            case SYS_MSG_BTN1_PRESSED:

               set_led_blink(LED_ID_1,
                            LED_ON_AT_BLINK_START,
                            52U, // blink_period, 
                            BLINK_INFINITE);

               break;

            case SYS_MSG_BTN1_RELEASED:

               set_led_off(LED_ID_1);
               led_cnt_dec();
 
               break;

            case SYS_MSG_LTT_1_EVENT:
 
               start_job2();

               break;

            case SYS_MSG_JOB2_STOP:

               stop_job2();

               break;

            case SYS_MSG_WAKEUP_BTN0:

               // Reopen shell UART - back to the regular operations

               (void)bsp_uart_open(p_uart,
                                   NULL, // const char* name,
                                   0,    // int32_t flags,
                                   (long)CFG_UART1_38400_8N1_20MHZ);
               
               break;

            default:

               break;
         }    
      }
      else
      {
         if(rc == TERR_TIMEOUT)
         {
            // do some periodic job, if system is not in the deep sleep.
            // The presence of the timeout(a system timer is active) does not
            // affects the deep sleep conditions    
         }   
      }
   } 
}

//----------------------------------------------------------------------------
void task_io_func(void * par)
{
   for(;;)
   {
      // Just stub

      tn_task_sleep(256);

 //   TestPrint("Cnt: %u\r\n",cnt++);
   }   
}

//----------------------------------------------------------------------------
//  Runs in the idle task
//----------------------------------------------------------------------------
void idle_hook_func(void * par)
{
    // If there are no tasks starvation and the system falls to the lowest 
   // priority(idle) task - re-arm WDT and check deep sleep conditions

   wdt_restart(); 

   if(check_deep_sleep_cond() == TRUE)
   {
     // Stop RTOS ticks and 1 sec RTC update - inside
     switch_to_deep_sleep();
   }

   // Go to CPU sleep - any case
   for(;;)
   {
      __bis_SR_register(LPM3_bits + GIE);
      __no_operation();                         // For debugger 
      __delay_cycles(250);
   }
}

//----------------------------------------------------------------------------
// Job 2 - blink LED_2  2 times, pause 5 sec, blink LED_2 again 3 times
//----------------------------------------------------------------------------
static void start_job2(void)
{
   DEEPSLEEPDATA * dsd = get_deep_sleep_data();

  // Prevent deep sleep - counter inside set_led_blink() and user timer set active
 

   set_led_blink(LED_ID_2,
                 LED_ON_AT_BLINK_START,
                 128U, // blink_period, 
                 2);

   (void)tn_timer_set(&tmrJob2,    // Timer to Start/re-start 
                      TOUT_5_SEC,  // timeout for the first action triggered (in sys ticks)
                      0UL);        // Periodic action time(if not 0)(in sys ticks)
}

//----------------------------------------------------------------------------
void timer_job2_func(TN_TIMER * tmr, void * param)
{
   SYS_MSG msg= {0};

   msg.op_code = SYS_MSG_JOB2_STOP;

   (void)tn_mailbox_send(&mailboxSysOp,
                        &msg,
                        sizeof(SYS_MSG),
                        TN_NO_WAIT); 
}

//----------------------------------------------------------------------------
int ltt1_timer_handler(int timer_id, LTT_DT_PK lt_timestamp, void * data)
{
   SYS_MSG msg = {0};
   int rc;

   msg.op_code = SYS_MSG_LTT_1_EVENT;

   rc = tn_mailbox_send(&mailboxSysOp,
                        &msg,
                        sizeof(SYS_MSG),
                        TN_NO_WAIT); 
   return rc;
}

//----------------------------------------------------------------------------
static void stop_job2(void)
{
  // the counter inside set_led_blink() will prevent a deep sleep
   set_led_blink(LED_ID_2,
                 LED_ON_AT_BLINK_START,
                 128U, // blink_period, 
                 3);
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


