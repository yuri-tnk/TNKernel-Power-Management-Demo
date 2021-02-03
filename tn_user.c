/*

  TNKernel real-time kernel

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

  /* ver 3  */

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

extern volatile int tn_inside_int_cnt;

__raw  __interrupt void PORT2_ISR(void);      // Int N 44
__raw  __interrupt void DMA_ISR(void);         // Int N 50
__raw  __interrupt void RTC_ISR(void);
__raw  __interrupt void TIMER0_B0_ISR(void);   // Int N 59

static volatile unsigned char * var_PORT2;
static volatile unsigned char * var_rtc;
static volatile unsigned char * var_uart1;
static volatile unsigned char * var_timer_B0;

//----------------------------------------------------------------------------
// Processor specific routine
//
//----------------------------------------------------------------------------
void tn_cpu_int_enable()
{

 // The global MSP430X CPU interrupts (GIE) will be enable
 // after the finishing of the function tn_start_exe() 


   //-- OS ticks interrupt source - RTC prescaler (div by 128)

   RTCPS0CTL &= (unsigned int)(~RT0PSIE);

     // RT0IP(bits 4..2) = 110b = Divide by 128; 32768Hz/128 = 256 Hz

   RTCPS0CTL |= (unsigned short)(6U<<2U);
   RTCPS0CTL |= (unsigned short)RT0PSIE;    //!< enable OS tick interrupt 

}

//============================================================================
// Hardware interrupts handlers
//============================================================================

//----------------------------------------------------------------------------
__task  void do_PORT2_ISR(void);

//----------------------------------------------------------------------------
#pragma vector = PORT2_VECTOR
__raw  __interrupt void PORT2_ISR(void)  // Int N 44
{
   tn_int_enter(var_PORT2);

   do_PORT2_ISR();

   tn_int_exit(var_PORT2);
}

__task  void do_PORT2_ISR(void)
{
   SYS_MSG msg;
   int rc;
 
   switch (__even_in_range(P2IV, P2IV_P2IFG7))
   {  
       //!< Vector  P2IV_NONE:  No Interrupt pending
      case  P2IV_NONE:
        
         break;
      //!< Vector  P2IV_P2IFG0:  P2IV P2IFG.0
      case  P2IV_P2IFG0: //!< P2.0 - Button 0
        
         P2IFG &= ~BIT0; //!< Clear int flag        
         P2IE  &= ~BIT0; //!< Disable P2.0 interrupt


         msg.op_code = SYS_MSG_INT_BTN0;

         rc = tn_mailbox_send(&mailboxUserHiPri,
                              &msg,
                              sizeof(SYS_MSG),
                              TN_NO_WAIT); 
         if(rc != TRUE)
         {
            // ToDo
         }

         break;

        //!< Vector  P2IV_P2IFG1:  P2IV P2IFG.1
      case  P2IV_P2IFG1:  //!< P2.1 - Button 1

         P2IFG &= ~BIT1; //!< Clear int flag        
         P2IE  &= ~BIT1; //!< Disable P2.1 interrupt

         break;
         
      default:
        
         break;   
   }
}

//----------------------------------------------------------------------------
__task  void do_DMA_ISR(void);

#pragma vector = DMA_VECTOR
__raw  __interrupt void DMA_ISR(void)  // Int N 50
{
   tn_int_enter(var_uart1);

   do_DMA_ISR();

   tn_int_exit(var_uart1);
}

__task  void do_DMA_ISR(void)
{
   UARTDS * pdata;

   switch(__even_in_range(DMAIV,16))
   {
      case  0:                           // No interrupt

         break;

      case  2:                                // DMA0IFG

#if defined USE_UART_1

         pdata = &g_p_uart_ds->g_uart1_ds;

       //  DMA0CTL &= ~(DMAEN | DMAIE);
         pdata->tx_buf = NULL;
            
         (void)tn_sem_signal(&semTxUart1);
#endif
         break;

      case  4:                           // DMA1IFG
         break;
      case  6:                           // DMA2IFG

#if defined USE_UART_2

         pdata = &g_p_uart_ds->g_uart2_ds;

       //  DMA2CTL &= ~(DMAEN | DMAIE);
         pdata->tx_buf = NULL;
            
         (void)tn_sem_signal(&semTxUart2);
#endif
         break;

      case  8:                           // DMA3IFG
         break;

      case 10:                           // DMA4IFG

#if defined USE_UART_0

         pdata = &g_p_uart_ds->g_uart0_ds;

       //  DMA4CTL &= ~(DMAEN | DMAIE);
         pdata->tx_buf = NULL;
            
        (void)tn_sem_signal(&semTxUart0);
#endif

         break;

      case 12:                           // Reserved
         break;
      case 14:                           // Reserved
         break;
      case 16:                           // Reserved
         break;
      default:   
         break;
   }
}

//----------------------------------------------------------------------------
__task  void do_RTC_ISR(void);

#pragma vector = RTC_VECTOR
__raw __interrupt void RTC_ISR(void)    // Int N 42
{
   tn_int_enter(var_rtc);

   do_RTC_ISR();

   tn_int_exit(var_rtc);
}

__task void do_RTC_ISR(void)
{
   SYS_MSG msg;
   unsigned int rv;
   int rc;

   while((BAKCTL & (unsigned int)LOCKBAK) == (unsigned int)LOCKBAK)            //!< Unlock backup system
   {
      BAKCTL &= (unsigned int)(~((unsigned int)LOCKBAK)); 
   }
//P4OUT |= BIT6;

   rv = __even_in_range(RTCIV, 14);
   switch(rv)
   {
      case  0:                            //!< Vector  0:  No interrupt
        
         break;                           
         
      case  2:                            //!< Vector  2:  RTCRDYIFG
       
        // g_sys_tick_1_sec++;

       //!< Just read raw data to prtc->op_dt and notify appropriate task
         
         g_prtc->op_dt->tm_sec  = RTCSEC;
         g_prtc->op_dt->tm_min  = RTCMIN;
         g_prtc->op_dt->tm_hour = RTCHOUR;
         g_prtc->op_dt->tm_wday = RTCDOW;
         g_prtc->op_dt->tm_mday = RTCDAY;
         g_prtc->op_dt->tm_mon  = RTCMON;
         g_prtc->op_dt->tm_year = RTCYEAR;
         
         msg.op_code = SYS_MSG_RTC_1SEC;

         rc = tn_mailbox_send(&mailboxUserHiPri,
                              &msg,
                              sizeof(SYS_MSG),
                              TN_NO_WAIT); 
         if(rc != TRUE)
         {
            // ToDo
         }

         break;
         
      case  4:    //!< Vector  4:  RTCEVIFG
        
         break;
         
      case  6:    //!< Vector  6:  RTCAIFG - RTC alarm

         RTCCTL0 &= (unsigned char)(~RTCAIFG);        

         msg.op_code = SYS_MSG_RTC_ALARM;

         rc = tn_mailbox_send(&mailboxUserHiPri,
                              &msg,
                              sizeof(SYS_MSG),
                              TN_NO_WAIT); 
         if(rc != TRUE)
         {
            // ToDo
         }

         break;
         
      case  8:    //!< Vector  8:  RT0PSIFG - RTOS tick 256 Hz (3.91 ms)

         tn_tick_int_processing();

         break;
         
      case 10:    //!< Vector 10:  RT1PSIFG
        
         break;
         
      case 12:    //!< Vector 12:  RTCOFIFG    32-kHz crystal oscillator fault interrupt flag.

         break;
         
      case 14:    //!< Vector 14:  Reserved
        
         break; 
         
      default: 
        
         break;
   }
}

//----------------------------------------------------------------------------
__task  void do_TIMER0_B0_ISR(void);

//----------------------------------------------------------------------------
#pragma vector = TIMER0_B0_VECTOR
__raw  __interrupt void TIMER0_B0_ISR(void)  // Int N 59
{
   tn_int_enter(var_timer_B0);

   do_TIMER0_B0_ISR();

   tn_int_exit(var_timer_B0);
}

//----------------------------------------------------------------------------
__task  void do_TIMER0_B0_ISR(void)
{
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

