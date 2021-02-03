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
#include "tn_utils.h"
#include "tn_port.h"
#include "prj_conf.h"
#include "types.h"
#include "uart.h"
#include "dt.h"
#include "func_prototypes.h"
#include "data_externals.h"


static BOOL is_rtc_1_sec_int_disabled(void);
static BOOL is_OS_ticks_disabled(void);
static void enable_rtc_1_sec_int(void);
static void enable_OS_ticks(void);
static void system_to_op_mode(void);

static void sys_shutdown_FS(void);
static void sys_shutdown_SD_card(void);
static void sys_shutdown_ADC(void);
static void sys_set_deep_sleep_pins(void);

static int cpu_set_clk_src_and_fll(unsigned short out_freq,  //!< desired system frequency (in kHz)
                                   unsigned long in_freq,    //!< crystal freq (in Hz)
                                   int osc_src);             //!< OSC_XT1 or OSC_XT2

static void change_cpu_freq(void);
static int chk_freq_conditions(void);

//----------------------------------------------------------------------------
void timer_dummy_func(TN_TIMER * tmr, void * param)
{
   // Do nothing here
}

//----------------------------------------------------------------------------
BOOL check_deep_sleep_cond(void)
{
   BOOL rc = FALSE;
   DEEPSLEEPDATA * dsd = get_deep_sleep_data();

   if(dsd->curr_osc == OSC_XT1 &&
        dsd->UART_cnt == 0 &&
              dsd->led_op_cnt == 0)
   {
       // user timers (not ltt timers !)
      if(is_timers_lists_empty() == TRUE) // no active timers
      {
         // Tasks have no job requests  
         if(is_pending_in_mailbox(&mailboxSysOp, &task_SysOp) == TRUE &&
            is_pending_in_mailbox(&mailboxUserHiPri, &task_user_hipri) == TRUE)
         {

            if(get_sem_cnt(&semTimerLT) > 0 &&
                get_sem_cnt(&semCPUFreq) > 0 &&
               //    get_sem_cnt(&semTestPrint) > 0 &&  Why commented ?
                   get_sem_cnt(&semRTC) > 0)
            {
               rc = TRUE;
            }
         }  
      }
   } 

   return rc;
} 

//----------------------------------------------------------------------------
void switch_to_op_mode(unsigned int param)
{
   unsigned long timeout;
  
   // Check if we are wakeup from deep sleep mode - In this case Int RTC 1 sec and
   // RTOS tick are disabled

   if(is_rtc_1_sec_int_disabled() == TRUE &&
       is_OS_ticks_disabled() == TRUE)
   {
     //-- To prevent imm go back to the deep sleep 

      if(param == SYS_MSG_RTC_ALARM)  // 10 Sec !!!
      {
         timeout = TOUT_10_SEC; 
      }
      else if(param == SYS_MSG_INT_BTN0)  // 20 Sec !!!
      {
         timeout = TOUT_20_SEC; 
      }
      else  // 10 ticks ~ 39.6 mS
      { 
         timeout = TOUT_10_SEC; 
      }

      (void)tn_timer_set(&tmrPreventDS, // Timer to Start/re-start 
                         timeout,       // timeout for the first action triggered (in sys ticks)
                         0UL);          // Periodic action time(if not 0)(in sys ticks)

      (void)rtc_get_clock_bcd_async();  // Update RTC after deep sleep

      system_to_op_mode();

      enable_rtc_1_sec_int();
      enable_OS_ticks();
   } 
}

//----------------------------------------------------------------------------
void switch_to_deep_sleep(void)
{
   TN_INTSAVE_DATA
 
   sys_shutdown_FS();
   sys_shutdown_SD_card();
   sys_shutdown_ADC();

   tn_disable_interrupt();

   sys_set_deep_sleep_pins();

   //!< enable button(s) interrupt

       //!< P2.0 - Button 0
        
   P2DIR &= ~BIT0;    
   P2IES |=  BIT0;  //!< Int on falling edge
   P2IFG &= ~BIT0;  //!< Clear int flag        
   P2IE  |=  BIT0;  //!< Enable P2.0 interrupt

#if 0      
       //!< P2.1 - Button 1
        
   P2DIR &= ~BIT1;    
   P2IES |=  BIT1;  //!< Int on falling edge
   P2IFG &= ~BIT1;  //!< Clear int flag        
   P2IE  |= BIT1;   //!< Enable P2.1 interrupt
#endif        

   //<! disable RTC 1 sec interrupt 
      
   RTCCTL01 &= ~RTCRDYIE;
      
   //!< disable OS tick interrupt 
      
   RTCPS0CTL &= (unsigned int)(~RT0PSIE);

   tn_enable_interrupt();

   wdt_set_long();   //-- Also re-arm WDT
}


//----------------------------------------------------------------------------
void UART_op_inc(void)
{
   DEEPSLEEPDATA * dsd = get_deep_sleep_data();

   dsd->UART_cnt++;

   change_cpu_freq();
}

//----------------------------------------------------------------------------
void UART_op_dec(void)
{
   DEEPSLEEPDATA * dsd = get_deep_sleep_data();
   if(dsd->UART_cnt > 0)
   {
      dsd->UART_cnt--;
      if(dsd->UART_cnt == 0)
      {
         change_cpu_freq();
      }
   }
}

//----------------------------------------------------------------------------
static BOOL is_rtc_1_sec_int_disabled(void)
{
   BOOL rc = FALSE;
   if((RTCCTL01 & RTCRDYIE) == 0U)  //<! RTC 1 sec interrupt is disabled
   {
      rc = TRUE; 
   }
   return rc;
}

//----------------------------------------------------------------------------
static BOOL is_OS_ticks_disabled(void)
{
   BOOL rc = FALSE;
   if((RTCPS0CTL & RT0PSIE) == 0U)  //!<  OS tick interrupt is disabled
   {
      rc = TRUE; 
   }
   return rc;
}

//----------------------------------------------------------------------------
static void enable_rtc_1_sec_int(void)
{
   RTCCTL01  |= RTCRDYIE;   //!< enable RTC 1 sec interrupt 
}

//----------------------------------------------------------------------------
static void enable_OS_ticks(void)
{
   RTCPS0CTL |= RT0PSIE;    //!< enable OS tick interrupt 
}

//----------------------------------------------------------------------------
static void system_to_op_mode(void)
{
   //!< P2.0 - Button 0
           
   P2IFG &= ~BIT0;  //!< Clear int flag        
   P2IE  &= ~BIT0;  //!< Disable P2.0 interrupt

  // Re-enable ADC, SD etc.

}

//----------------------------------------------------------------------------
static void sys_shutdown_FS(void)
{
  // stub -just for example
}

//----------------------------------------------------------------------------
static void sys_shutdown_SD_card(void)
{
  // stub -just for example
}

//----------------------------------------------------------------------------
static void sys_shutdown_ADC(void)
{
  // stub -just for example
}

//----------------------------------------------------------------------------
static void sys_set_deep_sleep_pins(void)
{
  // stub -just for example
}

//----------------------------------------------------------------------------
// when opening and closing UART, SD card etc.
//----------------------------------------------------------------------------

static void change_cpu_freq(void)
{
   int rc;
   rc = chk_freq_conditions();
   if(rc != TERR_NO_ERR)
   {
     //ToDo - Reset???
   }
}

//----------------------------------------------------------------------------
static int chk_freq_conditions(void)
{
   int rc = TERR_NO_ERR;
   DEEPSLEEPDATA * dsd = get_deep_sleep_data();

   if(//dsd->SD_files_cnt > 0 || 
        //dsd->SD_direct_cnt > 0 ||
          dsd->UART_cnt > 0)
   {
      rc = do_set_cpu_freq(SYS_CPU_FREQ_HIGH);
   } 
   else if(//dsd->SD_files_cnt == 0 &&
             //dsd->SD_direct_cnt == 0 &&
               dsd->UART_cnt == 0)
   {
      rc = do_set_cpu_freq(SYS_CPU_FREQ_LO);
   }

   return rc;
}

//----------------------------------------------------------------------------
int do_set_cpu_freq(int mode)
{
   int dv;
   int curr_osc_prev;
   int rc = -1;

   DEEPSLEEPDATA * dsd = get_deep_sleep_data();

   if(mode == SYS_CPU_FREQ_LO)
   {
      if(dsd->cpu_freq_l == SYS_CPU_FREQ_HIGH)
      {  
        // tn_disable_interrupt();
         curr_osc_prev = dsd->curr_osc;
         dsd->curr_osc = 0; // //!< Set as undefined - prevent deep sleep
        // tn_enable_interrupt();

         if(tn_system_state == TN_ST_STATE_RUNNING) 
         {
            rc = tn_sem_acquire(&semCPUFreq, TOUT_1_SEC); 
         }
         else
         {
            rc = TERR_NO_ERR;
         }

         if(rc == TERR_NO_ERR)
         {
            dsd->cpu_freq_l   = SYS_CPU_FREQ_LO;
            dsd->cpu_freq_val = SYS_FREQ_5MHZ;      
            
            dv = cpu_set_clk_src_and_fll(dsd->cpu_freq_val,  //!< desired system frequency (in kHz)
                                         OSC_X1_32768HZ,              //!< crystal freq (in Hz)
                                         OSC_XT1);                    //!< OSC_XT1 or OSC_XT2
            //!< delay to set freq
            
            if(tn_system_state == TN_ST_STATE_RUNNING) 
            {
               (void)tn_task_sleep(40); //!< ~160 ms             
            }
            else
            {
               while (dv--) 
               {
                  __delay_cycles(30); 
               }
            }

            stop_XT2_osc();
            // tn_disable_interrupt();
            dsd->curr_osc = OSC_XT1;
            // tn_enable_interrupt();

            if(tn_system_state == TN_ST_STATE_RUNNING) 
            {
               tn_sem_signal(&semCPUFreq); 
            }
         }
         else
         {
            // tn_disable_interrupt();
            dsd->curr_osc = curr_osc_prev;
            // tn_enable_interrupt();
         }
      } 
   }
   else if(mode == SYS_CPU_FREQ_HIGH)
   {
      if(dsd->cpu_freq_l == SYS_CPU_FREQ_LO)
      {  
        // tn_disable_interrupt();
         curr_osc_prev = dsd->curr_osc;
         dsd->curr_osc = 0; // //!< Set as undefined - prevent deep sleep
        // tn_enable_interrupt();

         if(tn_system_state == TN_ST_STATE_RUNNING) 
         {
            rc = tn_sem_acquire(&semCPUFreq, TOUT_1_SEC); 
         }
         else
         {
            rc = TERR_NO_ERR;
         }

         if(rc == TERR_NO_ERR)
         {
            dsd->cpu_freq_l   = SYS_CPU_FREQ_HIGH;
            dsd->cpu_freq_val = SYS_FREQ_20MHZ;      
            
            //!< set_XT2_osc() - inside bsp_set_clk_src_and_fll()  
            dv = cpu_set_clk_src_and_fll(dsd->cpu_freq_val, //!< desired system frequency (in kHz)
                                         OSC_X2_20MHZ,               //!< crystal freq (in Hz)
                                         OSC_XT2);                   //!< OSC_XT1 or OSC_XT2
            //!< delay to set freq
            
            //if(p_g_pwr_info->is_rtos_started) 
            if(tn_system_state == TN_ST_STATE_RUNNING) 
            {
               (void)tn_task_sleep(10); //!< ~40 ms             
            }
            else
            {
               while (dv--) 
               {
                  __delay_cycles(30); 
               }
            }
            // tn_disable_interrupt();
            dsd->curr_osc = OSC_XT2;
            // tn_enable_interrupt();

            if(tn_system_state == TN_ST_STATE_RUNNING) 
            {
               tn_sem_signal(&semCPUFreq); 
            }
         }
         else
         {
            // tn_disable_interrupt();
            dsd->curr_osc = curr_osc_prev;
            // tn_enable_interrupt();
         }
      } 
   }

   return rc;
}

//----------------------------------------------------------------------------
static int cpu_set_clk_src_and_fll(unsigned short out_freq,  //!< desired system frequency (in kHz)
                                   unsigned long in_freq,    //!< crystal freq (in Hz)
                                   int osc_src)              //!< OSC_XT1 or OSC_XT2
{
   unsigned short ratio;
   unsigned int dv;
   unsigned int dco_div_bits;

   ratio = (unsigned short)(((unsigned long)out_freq * 1000UL)/in_freq);
 
   if(osc_src == OSC_XT2)          //!< out_freq will be 20 MHz 
   {
      P7DIR &= ~(BIT2 + BIT3);     //!< P7.2,P7.3 -inputs

      P7SEL |= BIT2 + BIT3;        //!< Port select XT2
  
      UCSCTL6 &= ~XT2OFF;          //!< Start XT2
  
      while (SFRIFG1 & OFIFG) 
      {                 // Check OFIFG fault flag
         UCSCTL7 &= (unsigned int)(~(DCOFFG + XT1LFOFFG + XT1HFOFFG + XT2OFFG)); // Clear OSC flaut Flags
         SFRIFG1 &= (unsigned int)(~OFIFG);                      // Clear OFIFG fault flag
      }

      UCSCTL4 =   (0x5 << 4) |     //!< SELS Bits 6:4=101 - XT2OSC (Selects the SMCLK source)
                          5;       //!< SELM Bits 2:0=101 - XT2OSC (Selects the MCLK source)

    // P3DIR |= BIT4;
    // P3SEL |= BIT4;
   }
   else //!< OSC_XT1
   {
     
      dv = ratio;
      if(out_freq > 16000U)
      {
         dco_div_bits = FLLD__2;  //!< Have at least a divider of 2
         dv >>= 1 ;
      }
      else
      {
         dco_div_bits = FLLD__2;  //!< Have at least a divider of 2
      }

      while(dv > 512)
      {
         dco_div_bits++;
         dv >>= 1;
      }

      UCSCTL0 = 0;                //!< Set DCO to lowest Tap

      UCSCTL3 = 0;                //!< SELREF = 0 (XT1CLK); FLLREFDIV = 0 (f(FLLREFCLK)/1)

      if(out_freq == 1000)        //!< Set DCO Multiplier for 8MHz
      {                           //!< (N + 1) * FLLRef = Fdco
          UCSCTL2 = 29;           //!< (249 + 1) * 32768 = 8MHz   
      }
      else  
      { 
         UCSCTL2 = dco_div_bits | (dv - 1);
      }
   
      if(out_freq <= 630U)        //!< out_freq < 0.63MHz
      {
         UCSCTL1= DCORSEL_0 ;
      }
      else if(out_freq < 1250U)   //!< 0.63MHz < out_freq < 1.25MHz
      {
         UCSCTL1= DCORSEL_1 ;
      }
      else if(out_freq < 2500U)   //!< 1.25MHz < out_freq < 2.5MHz
      {
         UCSCTL1= DCORSEL_2 ;
      }
      else if(out_freq < 5000U)   //!< 2.5MHz < out_freq < 5MHz
      {
         UCSCTL1= DCORSEL_3 ;
      }
      else if(out_freq < 10000U)  //!< 5MHz < out_freq < 10MHz
      {
         UCSCTL1= DCORSEL_4 ;
      }
      else if(out_freq < 20000U)  //!< 10MHz < out_freq < 20MHz
      {
         UCSCTL1= DCORSEL_5 ;
      }
      else if(out_freq < 40000U)  //!< 20MHz < out_freq < 40MHz
      {
         UCSCTL1= DCORSEL_6 ;
      }
      else
      {
         UCSCTL1= DCORSEL_7 ;
      }

      do
      {
         UCSCTL7 &= (unsigned int)(~(XT2OFFG + XT1LFOFFG + DCOFFG));            //!< Clear XT2,XT1,DCO fault flags
         SFRIFG1 &= (unsigned int)(~OFIFG);                     //!< Clear fault flags
      }while(SFRIFG1 & OFIFG);                                  //!< Test oscillator fault flag
 
   //!< Assign the clock (MCLK, SMCLK) sources
   //!< Case SELA (bits 10-8) = 0, ACLK is always XT1CLK

     UCSCTL4 =   (0x4 << 4) |     //!< SELS Bits 6:4=100 - DCOCLKDIV (Selects the SMCLK source)
                         4;       //!< SELM Bits 2:0=100 - DCOCLKDIV (Selects the MCLK source)
   }
   
   //!< Wait until FLL clock will be stabilized - outside of the function
   
   dv = ratio<<5;  //!< * 32;       

   return (int)dv;
}

//----------------------------------------------------------------------------
void led_cnt_inc(void)
{
   g_deep_sleep_data.led_op_cnt++;
}

//----------------------------------------------------------------------------
void led_cnt_dec(void)
{
   if(g_deep_sleep_data.led_op_cnt > 0)
   {
      g_deep_sleep_data.led_op_cnt--;
   } 
}


//----------------------------------------------------------------------------
DEEPSLEEPDATA * get_deep_sleep_data(void)
{
   return &g_deep_sleep_data;
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

