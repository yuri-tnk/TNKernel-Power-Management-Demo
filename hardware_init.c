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

  /* ver 3  */


#include "io430.h"

#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "tn_config.h"
#include "tn.h"
#include "pmm.h"

#include "tn_utils.h"
#include "tn_port.h"
#include "prj_conf.h"
#include "types.h"
#include "uart.h"
#include "dt.h"
#include "func_prototypes.h"
#include "data_externals.h"


//-- Board LB3

void set_sys_clock(void);
void set_XT2_osc(void);
void set_32khz_osc(void);
uint16_t SetVCore (uint8_t level);
void sys_init_cpu(void);
void do_rtc_init(void);


  //!< Local prototypes

void init_cpu_pins_after_reset(void);
void bsp_set_storage_mode_pins_test(void);
void bsp_set_deep_sleep_pins(void);

/*--- Valid only for the project:  

 Port   Pin                              Usage                                        Dir     State after Reset Init
------ ----- --------------------------------------------------------------        -------   -----------------------  

 P6.4 +   1  M_AN_SPARE_1   S_AN_1( J5 pin  9)  - not used                            IN        IN
 P6.5 +   2  M_AN_SPARE_2   S_AN_2( J5 pin  10)  - not used                           IN        IN
 P6.6 +   3  M_AN_SPARE_3   S_AN_3( TP2 ) - not used                                  IN        IN + pull-up resistor
 P6.7 +   4  M_V24V_EN      0 - disable DC/DC 24 V  1 - enable                       OUT        '0'
 P7.4 +   5  M_VOUT_EN      1-enable 3.3V LDO for external consumers  0-dis          OUT        '0'
 P7.5 -+  6  M_SUPPBY_VDDn  GPRS/RS485 pwr from VDD (Li battery);  1-en 0-dis        OUT        '0'
 P7.6 -+  7  M_SUPPBY_EXTn  GPRS/RS485 pwr from V3P8 (ext voltag); 1-en 0-dis        OUT        '0' 
 P7.7     8  M_BUCK_EN      Ena U7 DC/DC converter (ext voltage to V3P8) 1-en 0-dis  OUT        '0'
 P5.0 +   9  M_IRD_EN       Ena power for U3 IRDA   1- en 0 - dis                    OUT        '0'
 P5.1 VV 10  M_SU_EN        Ena U9 DC/DC (solenoid power source) 1-en 0-dis          OUT        '0'
 P5.6    16  M_GSM_FSDN     GSM/GPRS modem FAST_SHTDWN - 1- active 0-not active      OUT        '0'
 P2.0    17  M_SW_0         Button SW1  0 - pressed 1 - not pressed                   IN        IN + pull-up resistor
 P2.1    18  M_SW_1         Button SW2  0 - pressed 1 - not pressed                   IN        IN + pull-up resistor
 P2.2 VV 19  M_EXT_GPIO2    not used                                                 OUT        '0'
 P2.3    20  M_DRV_IN_1     Solenoid drv ctrl IN1                                    OUT        '0'
 P2.4    21  M_DRV_IN_2     Solenoid drv ctrl IN2                                    OUT        '0'
 P2.5    22  M_DRV_EN       Solenoid drv ctrl SLEEPn                                 OUT        '0'
 P2.6    23  M_LED_BL       Background light LEDs  1-on 0-off                        OUT        '0'
 P2.7    24  M_HALLSW_PWR   Switch hall sensor power ( J1 pin 1)                     OUT        '1'
 P5.2 +  28  M_REn          RE ena for RS485 drv IC  0-en 1-dis                      OUT        '1'
 P5.3    31  M_GSM_EMERRST  GSM\GPRS modem reset    (1-active 0-not active)          OUT        '0'
 P5.4    32  M_DE           DE ena for RS485 drv IC (1-TX en 0-Tx in Z-state)        OUT        '0'
 P5.5 +  33  M_MULTI_AN_EN  Enable some analog meas (1-en 0-dis)                     OUT        '0'
 P1.0    34  M_HALL_SIG_0   Water stream Hall sensor 1 output (J4 pin 3)              IN        IN + pull-up resistor
 P1.1    35  M_HALL_SIG_1   Water stream Hall sensor 2 output (J4 pin 5)              IN        IN + pull-up resistor
 P1.2    36  M_HALLSW_SIG   Switch hall sensor input (J1 pin 3)                       IN        IN + pull-up resistor
 P1.3    37  M_TA0CC2_OUT   Timer A0 CC2 out-for the connection to Tmr 1             OUT        '0'
 P1.4    38  M_GSM_ON_IND   Input for the GSM\GPRS modem PWR ON signal                IN        IN
 P1.5    39  M_GSM_CTS0     GSM\GPRS modem CTS signal (modem - out CPU - in)          IN        IN + pull-up resistor
 P1.6    40  M_GSM_RTS0     GSM\GPRS modem RTS signal (modem - in  CPU - out)        OUT        '0'
 P1.7 -+ 41  M_GSM_RING0    GSM\GPRS modem RING signal(modem - out CPU - in)          IN        IN
 P3.0    42  M_TA1CLK       Timer A1 input  - for the conn to Timer A0 CC2 out        IN        IN + pull-up resistor
 P3.1 +  43  M_PG           DC/DC U7 power good (1-good 0-not yet)                    IN        IN
 P3.2    44  M_ALARM_IND    Ext signal S_ALARM (J5 pin 2)   (1-active)                IN        IN + pull-up resistor
 P3.3    45  Ext RS485  connected M_AN_SPARE_2/S_AN_2(J5 pin 10) 0-not conn 1-conn    IN        IN + pull-up resistor
 P3.4 VV 46  M_EXT_GPIO1    TP15 - not used                                          OUT        '1' 
 P3.5    47  M_FLS_GPIO_0   SD_IN_INDICATION   SD Card existance  0-inserted          IN        IN + pull-up resistor
 P3.6    48  M_HALL_PWM_0   PWM for water Stream Hall sensor 1                       OUT        '1'
 P3.7 +  49  M_HALL_PWM_1   PWM for water Stream Hall sensor 2                       OUT        '1'
 P4.0    50  M_LED0         LED0 ctrl  1- on 0 - off                                 OUT        '0'
 P4.1    51  M_LED1         LED1 ctrl  1- on 0 - off                                 OUT        '0'
 P4.2    52  M_LED2         LED2 ctrl  1- on 0 - off                                 OUT        '0'
 P4.3    53  M_LED3         LED3 ctrl  1- on 0 - off                                 OUT        '0'
 P4.4    54  M_LED4         LED4 ctrl  1- on 0 - off                                 OUT        '0'
 P4.5    55  M_LED5         LED5 ctrl  1- on 0 - off                                 OUT        '0'
 P4.6    56  M_LED6         LED6 ctrl  1- on 0 - off                                 OUT        '0'
 P4.7    57  M_LED7         LED7 ctrl  1- on 0 - off                                 OUT        '0'
 P8.0    58  M_FLS_VDD_ENn  SDCard/FLASH Power  0-On  1-Off                          OUT        '1'
 P8.1    59  M_GSM_IGT      GSM/GPRS modem IGT   1-active                            OUT        '0'
 P8.2    60  M_UART1_TX     RS485 UART TxD                                           OUT        '1'
 P8.3    61  M_UART1_RX     RS485 UART RxD                                            IN        IN + pull-up resistor
 P8.4    62  M_FLS_CLK      SDCard/FLASH CLK (UCB1 CLK)                              OUT        '1'
 P8.5    65  M_FLS_SIMO     SDCard/FLASH MOSI (UCB1 MOSI)                            OUT        '1'
 P8.6    66  M_FLS_SOMI     SDCard/FLASH MISO (UCB1 MISO)                             IN        IN + pull-up resistor
 P8.7    67  M_FLS_CSn      SDCard/FLASH  CS   0-selected 1- no op                   OUT        '1'
 P9.0 VV 68  M_SENS_EN      Enable 4..20 mA sensors resistors                        OUT        '0'
 P9.1    69  M_UCB2_CSn     reserved (TP21)                                          OUT        '0'
 P9.2    70  M_UART2_TX     GSM/GPRS modem TxD                                       OUT        '1'
 P9.3    71  M_UART2_RX     GSM/GPRS modem RxD                                        IN        IN + pull-up resistor
 P9.4    72  M_UCB2_CLK     reserved (TP6)                                           OUT        '0'
 P9.5    73  M_UCB2_SIMO    reserved (TP14)                                          OUT        '0'
 P9.6    74  M_UCB2_SOMI    reserved (TP20)                                          OUT        '0' 
 P9.7 VV 75  M_HALL_PWR     Power for the Water stream Hall sensor                   OUT        '1' 
 PU.0 +  77  not used                                                                    Default
 PU.1 +  79  not used                                                                    Default
 P5.7    88  M_RTCCLK                                                                    Default
 P6.0    97  M_AN_DRV_C     ADC A0 in - the solenoid valve current meas               IN         IN + pull-up resistor
 P6.1    98  M_AN_TEMP_V    ADC A1 in - the thermoresistor measurement                IN         IN + pull-up resistor
 P6.2    99  M_AN_EXTBAT_V  ADC A2 in - external power voltage measurement            IN         IN + pull-up resistor 
 P6.3   100  M_AN_INBAT_V   ADC A2 in - VDD (the Li battery) voltage measurement      IN         IN + pull-up resistor     
            
 ----------------------------------------------------------------------------*/

//----------------------------------------------------------------------------
void sys_init_cpu(void)
{
   //-- First, make sure the watchdog is disabled.

   //WDTCTL = WDTPW + WDTHOLD;           // Stop WDT

   (void)PMM_setVCore(PMM_BASE,      // Set VCore = 1.9V for 20MHz clock
                      PMMCOREV_3);

   init_cpu_pins_after_reset();
   
   P2OUT |= (unsigned char)BIT7;   //!< Enable power for HALL switch sensor

//---------------------------------------
   
   set_32khz_osc();

   do_rtc_init();

   g_deep_sleep_data.cpu_freq_l = SYS_CPU_FREQ_HIGH;
   do_set_cpu_freq(SYS_CPU_FREQ_LO); 
   
   //P3DIR |= BIT4;                            // P3.4 pin 46 SMCLK set out to pins
   //P3SEL |= BIT4;
}

//----------------------------------------------------------------------------
void set_32khz_osc(void)
{
   //!< Enable XT1
  
   while((BAKCTL & (unsigned short)LOCKIO) == (unsigned short)LOCKIO)                   //!< Unlock XT1 pins for operation
   {
      BAKCTL &= (unsigned int)(~((unsigned short)LOCKIO));
   }
   UCSCTL6 &= ~((unsigned short)XT1OFF);                     //!< XT1 On
   //UCSCTL6 |= XCAP_0;                      // Internal load cap 
   //UCSCTL6 |= XCAP_1;                      // Internal load cap 
   //UCSCTL6 |= XCAP_2;                      // Internal load cap 
   UCSCTL6 |= XCAP_3;                      //!< Internal load cap
  
   do
   {
      UCSCTL7 &= (unsigned int)(~((unsigned short)XT2OFFG + 
                                  (unsigned short)XT1LFOFFG /*+ XT1HFOFFG*/ + 
                                  (unsigned short)DCOFFG));
                                             //!< Clear XT2,XT1,DCO fault flags
      SFRIFG1 &= (unsigned int)(~OFIFG);     //!< Clear fault flags
   }while ((SFRIFG1 & (unsigned int)OFIFG) == (unsigned int)OFIFG);       //!< Test oscillator fault flag

   UCSCTL6 &= ~(XT1DRIVE_3);                 //!< Xtal is now stable, reduce drive
                                             //!< strength
//--- Fault 32 khz osc int flags 

   RTCCTL0 &= ~((unsigned char)RTCOFIFG);  // Clear int flag
   RTCCTL0 |= (unsigned char)RTCOFIE;  // Enable int

   UCSCTL4 &= (unsigned int)(~((unsigned int)SELA0 + 
                               (unsigned int)SELA1 + 
                               (unsigned int)SELA2));      //!< Ensure XT1 is ACLK source
}

//----------------------------------------------------------------------------
void do_rtc_init(void)
{
   unsigned char  val_uch = 1U;
//   unsigned short val_ush = 1U;
//--------------------------------------------------------

      //---- Disable Alarm --------------------
     
      RTCCTL0 &= (unsigned char)(~((unsigned char)RTCAIE | 
                                   (unsigned char)RTCAIFG)); //-- disable Alarm interrupt
      RTCAMIN  = 0x00;
      RTCAHOUR = 0x00;
      RTCADAY  = 0x00;
      RTCADOW  = 0x00;
      
      //---------------------------------------
      
   // Configure RTC_B - BCD calendar mode
   
      RTCCTL01 |= /*(unsigned short)RTCRDYIE |*/ // enable RTC read ready interrupt
                  (unsigned short)RTCBCD |   // BCD mode
                  (unsigned short)RTCHOLD;   // RTC hold 

      RTCSEC  = val_uch;
      RTCMIN  = val_uch;
      RTCHOUR = val_uch;
      RTCDOW  = val_uch;
      RTCDAY  = val_uch;
      RTCMON  = val_uch;
      RTCYEAR = (unsigned short)2000U;

      RTCCTL01 &= ~((unsigned short)RTCHOLD);  // Start RTC calendar mode
}

//----------------------------------------------------------------------------
//   Now it is set for the crystal 20 MHz
//----------------------------------------------------------------------------
void set_XT2_osc(void)  
{
   P7DIR &= ~((unsigned char)BIT2 + (unsigned char)BIT3);     //!< P7.2,P7.3 -inputs

   P7SEL |= (unsigned char)BIT2 + (unsigned char)BIT3;        //!< Port select XT2
  
   UCSCTL6 &= ~(unsigned short)XT2OFF;          //!< Start XT2
  
   while((SFRIFG1 & (unsigned short)OFIFG) != 0U)
   {                 // Check OFIFG fault flag
      UCSCTL7 &= (unsigned int)(~((unsigned int)DCOFFG + 
                                  (unsigned int)XT1LFOFFG +
                                  (unsigned int)XT1HFOFFG + 
                                  (unsigned int)XT2OFFG)); // Clear OSC flaut Flags
      SFRIFG1 &= ~((unsigned int)OFIFG);                    // Clear OFIFG fault flag
   }


   UCSCTL4 =   (unsigned short)(0x5U << 4) |     //!< SELS Bits 6:4=101 - XT2OSC (Selects the SMCLK source)
               (unsigned short)5;       //!< SELM Bits 2:0=101 - XT2OSC (Selects the MCLK source)

    // P3DIR |= BIT4;
    // P3SEL |= BIT4;
}

//----------------------------------------------------------------------------
void stop_XT2_osc(void)
{
  // UCSCTL4 = 0; //-- ACLK, SMCLK, MCLK = XT1CLK

   UCSCTL6 |=  (unsigned short)XT2OFF;                //!< Set XT2 Off
   
   P7SEL   &= ~((unsigned char)BIT2 + (unsigned char)BIT3);         //!< Port select XT2
   P7DIR   |=   (unsigned char)BIT2 + (unsigned char)BIT3;          //!< P7.2, P7.3 -outputs
   P7OUT   &= ~((unsigned char)BIT2 + (unsigned char)BIT3);         //!< P7.2, P7.3 - set to 0
}


//----------------------------------------------------------------------------
void init_cpu_pins_after_reset(void)
{

/*
  P1.0/TA0CLK/ACLK        34     M_HALL_SIG_0          IN            Output of the Water stream Hall sensor (Interrupt ???)
  P1.1/TA0.0              35     M_HALLSW_SIG          IN            Override Hall Sensor Input (J1/3, TP57)
  P1.2/TA0.1              36     CAPT_SIG1             IN/TA0CCR1    Uses as TA0CCR1 capture input
  P1.3/TA0.2              37     M_TA0CC2_OUT          OUT/TA0CCR2   Uses as TA0CCR2 output(out of the Timer A0)
  P1.4/TA0.3              38     M_GSM_ON_IND          IN            Input for the GSM\GPRS modem PWR ON signal
  P1.5/TA0.4              39     M_GSM_CTS0            IN/pull-down? GSM\GPRS modem CTS signal (modem - out CPU - in)
  P1.6/TA0.1              40     M_GSM_RTS0            OUT           GSM\GPRS modem RTS signal (modem - in  CPU - out)
  P1.7/TA0.2              41     M_GSM_RING0           IN            GSM\GPRS modem RING signal(modem - out CPU - in)
*/
      P1REN = BIT5;        //!< Enable pull resistor 
      P1OUT = 0U;          //!< Here BIT3, BIT6 set to 0; BIT5=0 ->pull-down resistor
      P1DIR = BIT3 | BIT6; // Outputs - P1.3,P1.6, Inputs P1.0, P1.1, P1.2, P1.4, P1.5, P1.7

/*
  P2.0/P2MAP0             17     M_SW_0                IN            Button SW1(TP35) 0 - pressed 1 - released 
  P2.1/P2MAP1             18     M_SW_1                IN            Button SW2(TP51) 0 - pressed 1 - released 
  P2.2/P2MAP2             19     M_EXT_GPIO2           OUT(IN/pull-up?) Ext GPIO2 (J5/2, TP14) Set as OUT '0'
  P2.3/P2MAP3             20     M_DRV_IN_1            OUT           Solenoid Drv Input 1 (CPU - '0')
  P2.4/P2MAP4             21     UART0_TX              OUT           Debug UART TX (J1/7, TP42)
  P2.5/P2MAP5             22     UART0_RX              IN/pull-down  Debug UART RX (J1/9, TP87), Interrupt
  P2.6/P2MAP6             23     M_DRV_IN_2            OUT           Solenoid Drv Input 2 (CPU - '0') 
  P2.7/P2MAP7             24     M_DRV_EN              OUT           Solenoid Drv ctrl SLEEPn (CPU - '0') 
*/
      P2REN = BIT5;        //!< Enable pull resistor 
      P2OUT = BIT4;        //!< Here P2.2, P2.3, P2.6, P2.7 set to 0, P2.4 set to 1; BIT5=0 ->pull-down resistor
      P2DIR = BIT2 | BIT3 | BIT4 | BIT6 | BIT7; // Inputs - P2.0,P2.1,P2.5 Outputs - P2.2, P2.3, P2.4, P2.6, P2.7

/*
  P3.0/TA1CLK/CBOU        42     M_TA1CLK              IN/TA1CLK     Uses as TA1CLK input
  P3.1/TA1.0              43     M_PG                  IN            Ext Power Good Input ( 1 - Ext Power Rdy 0 -Ext Power Not Rdy)
  P3.2/TA1.1              44     CAPT_SIG2             IN/TA1CCR1    Uses as TA1CCR1 capture input 
  P3.3/TA1.2              45     Not used              OUT           '0' 
  P3.4/TA2CLK/SMCLK       46     M_EXT_GPIO1           OUT           ext "Open Valve" button (J1/4, TP15) '0'
  P3.5/TA2.0              47     M_ALARM_IND           IN/pool-down  Ext signal J_ALARM (J5/1) with level converter, 1-active 0-not active)
  P3.6/TA2.1              48     M_HALL_PWM_0          OUT           PWM for water Stream Hall sensor '1' 
  P3.7/TA2.2              49     WD_RST_IRQ            IN            Interrupt if Ext WDT fired
*/
      P3REN = BIT4 | BIT5;      // !< Enable pull resistors 
      P3OUT |= BIT4;            // !< Here P3.4 is set to 1 and it is pullup resistor, 
      P3DIR = BIT3 /*| BIT4*/ | BIT6;   // Inputs - P3.0,P3.1,P3.2, P3.4, P3.5, P3.7  Outputs - P3.3, P3.4, P3.6
      P3OUT |= BIT6;            //  P3.6 set to 1; BIT5=0 ->pull-down resistor

/*
  P4.0/TB0.0              50     M_LED_0               OUT           LED LD4        ("OPEN")
  P4.1/TB0.1              51     M_LED_1               OUT           LED LD12 RED   ("LEAK")
  P4.2/TB0.2              52     M_LED_2               OUT           LED LD12 GREEN ("LEAK")
  P4.3/TB0.3              53     M_LED_3               OUT           LED LD17       ("NORMAL")
  P4.4/TB0.4              54     M_LED_4               OUT           LED LD20       ("OVERRIDE")
  P4.5/TB0.5              55     M_LED_5               OUT           LED LD18       ("AWAY")
  P4.6/TB0.6              56     M_LED_6               OUT           LED LD24 RED   ("ERROR")
  P4.7/TB0OUTH/SVMOUT     57     M_LED_7               OUT           LED LD24 GREEN ("ERROR")
*/
      P4OUT = 0U;     // All - set to '0'
      P4DIR = 0xFFU;  // All -as outputs
/*
  P5.0/VREF+/VeREF+        9     M_KICK                OUT           "Kick" Ext WDT (1 - active) 
  P5.1/VREF-/VeREF-       10     M_SU_EN               OUT           Solenoid DC-DC ena (1- enable)
  P5.2                    28     M_REn                 OUT           RS485 Driver REn signal
  P5.3                    31     M_GSM_EMERRST         OUT           GSM\GPRS modem reset (1-active 0-not active)
  P5.4                    32     M_DE                  OUT           RS485 Driver DE signal (1-TX en 0-Tx in Z-state)  '0'
  P5.5                    33     M_MULTI_AN_EN         OUT           Enable a few analog measurements (1-en 0-dis) 
  P5.6/ADC12CLK/DMAE0     16     M_GSM_FSDN            OUT           GSM/GPRS modem FAST_SHTDWN (1- active 0-not active)
  P5.7/RTCCLK             88     M_RTCCLK              Sys           Clock Output
*/
      P5OUT |= BIT2;   //  Set P5.2 to 1, all others(except P5.7 ) - to 0
      P5DIR |= 0xEFU;  // All -as outputs (leave P5.7 at default setting)

/*
  P6.0/CB0/A0             97     M_AN_DRV_C            IN/A0         Solenoid current(not plunger) ADC input
  P6.1/CB1/A1             98     M_AN_TEMP_V           IN/A1         Temeperature resistor ADC input 
  P6.2/CB2/A2             99     M_AN_EXTBAT_V         IN/A2         Ext battery (VEXT12) voltage measurement ADC input  
  P6.3/CB3/A3            100     M_AN_INBAT_V          IN/A3         VBAT voltage measurement ADC input  
  P6.4/CB4/A4              1     M_AN_SPARE_1          IN/A4         S_AN_1 (J5/9) - 4..20 mA sensor N 1
  P6.5/CB5/A5              2     M_AN_SPARE_2          IN/A5         S_AN_2 (J5/7) - 4..20 mA sensor N 2
  P6.6/CB6/A6/DAC0         3     M_AN_PLUNG            IN/A6         Solenoid plunger position ADC input
  P6.7/CB7/A7/DAC1         4     M_18V_EN              OUT           DC_DC 18V Ena (1 - Enable 0 - Disable)
*/
      P6OUT = 0U;    //  P6.7 = 0
      P6DIR = 0x80;  // All -as inputs, except P6.7
/*
  *P7.2/XT2IN             84     M_XT2_IN              Sys           Crystal 20 MHz
  *P7.3/XT2OUT            85     M_XT2_OUT             Sys           Crystal 20 MHz
  P7.4/CB8/A12             5     M_VOUT_EN             OUT           DC_DC VOUT_3V3 enable ( 1- enable 0- disable) '0'
  P7.5/CB9/A13             6     M_SUPPBY_VDDn         OUT           Vmodem_from_battery enable (1- enable 0- disable) '0'  
  P7.6/CB10/A14/DAC0       7     M_SUPPBY_EXTn         OUT           Vmodem_from_Ext_4V_DC_DC enable (1- enable 0- disable) '0'  
  P7.7/CB11/A15/DAC1       8     M_LOC_DSBL            OUT           Solenoid plunger position Current sensor(1-disable 0-enable)
*/
      P7OUT &= ~(BIT4 | BIT5 | BIT6);       // P7.4, P7.5, P7.6 - set to '0'
      P7OUT |=  BIT7;                       // P7.7 - set to '1'
      P7DIR =   BIT4 | BIT5 | BIT6 | BIT7;  // P7.4, P7.5, P7.6, P7.7 - outputs

/*
  P8.0/TB0CLK             58     M_FLS_VDD_ENn         OUT           SD Card Power (0-enable 1-disable)
  P8.1/UCB1STE/UCA1CLK    59     M_GSM_IGT             OUT           GSM\GPRS modem "ignition" (1-active 0-not active)
  P8.2/UCA1TXD/UCA1SIMO   60     M_UART1_TX            OUT/UART      RS485 UART TxD (DI) 
  P8.3/UCA1RXD/UCA1SOMI   61     M_UART1_RX            IN/UART       RS485 UART RxD (RO)
  P8.4/UCB1CLK/UCA1STE    62     M_FLS_CLK             OUT/UCB1CLK   Also SD_CLK
  P8.5/UCB1SIMO/UCB1SDA   65     M_FLS_SIMO            OUT/UCB1SIMO  Also SD_SIMO
  P8.6/UCB1SOMI/UCB1SCL   66     M_FLS_SOMI            IN/UCB1SOMI   Also SD_SOMI
  P8.7                    67     M_FLS_CSn             OUT           Also SD_CSn
*/
  // For SD card -  P8.0 = 1 - SD power off
  //
  //                P8.4 = 0   62  M_FLS_CLK   
  //                P8.5 = 0   65  M_FLS_SIMO   
  //                P8.6 = 0   66  M_FLS_SOMI     
  //                P8.7 = 0   67  M_FLS_CSn      

      P8SEL = 0;             //  Set as regular I/O  
      UCB1CTL1 |=  UCSWRST;  //  Reset USCI state machine

      P8OUT &=  ~(BIT1 | BIT4 | BIT5 | BIT6 | BIT7);       //!< Out-> to '0'
      P8OUT |=    BIT0 | BIT2;                             //!< Out-> to '1' 
      P8DIR  =    BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT7; //!< OUT
/*
  P9.0                    68     M_LED_8               OUT           LED LD5 ("CLOSED")
  P9.1/UCB2STE/UCA2CLK    69     M_EXT_GPIO3           OUT           EXT_GPIO3  (J5/6, TP21) '0'
  P9.2/UCA2TXD/UCA2SIMO   70     M_UART2_TX            OUT/UCA2TXD   GSM/GPRS modem TxD
  P9.3/UCA2RXD/UCA2SOMI   71     M_UART2_RX            IN/UCA2RXD    GSM/GPRS modem RxD
  P9.4/UCB2CLK/UCA2STE    72     M_RAM_CS              OUT           CS SPI RAM ('1')
  P9.5/UCB2SIMO/UCB2SDA   73     M_LED_BL              OUT           Backlight LED (1- on 0-off)  
  P9.6/UCB2SOMI/UCB2SCL   74     M_HALLSW_PWR          OUT           Override Hall Sensor Power (J1/1, TP61)
  P9.7                    75     M_HALL_PWR            OUT           Water stream Hall Sensor Power
*/
      P9OUT &= ~(BIT0 | BIT1 | BIT5);                           // Set P9.0, P9.1, P9.5, 
      P9OUT |= BIT2 | BIT4 | BIT6 | BIT7;                       // Set P9.2, P9.4, P9.6, P9.7 - to '1'
      P9DIR  = BIT0 | BIT1 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7;  // All output, except P9.3
/*
  PU.0                    76     Not used
  PU.1                    79     Not used

  PJ.3/TCK                95     M_TCK     CPU internal
  PJ.2/TMS                94     M_TMS     CPU internal
  PJ.1/TDI/TCLK           93     M_TDI     CPU internal
  PJ.0/TDO                92     M_TDO     CPU internal
  TEST/SBWTCK             91     M_TEST    CPU internal

  RST/NMI/SBWTDIO         96     M_RESET   CPU internal
*/

}


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
