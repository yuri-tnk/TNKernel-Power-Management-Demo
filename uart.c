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

 /* ver 3.0 */

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


UARTSINFO * bsp_get_uart_config_info_addr(void);
UARTDS * bsp_get_uart_data_struct_addr(uint32_t num_uart);
unsigned char * bsp_get_uart_rx_buf(int32_t num_uart);

#if defined USE_UART_0     
extern TN_SEM semTxUart0;
extern TN_SEM semTxBufUart0;
#endif

#if defined USE_UART_1     
extern TN_SEM semTxUart1;
extern TN_SEM semTxBufUart1;
#endif

#if defined USE_UART_2     
extern TN_SEM semTxUart2;
extern TN_SEM semTxBufUart2;
#endif

extern const UARTSINFO g_uart_config_info[];
const UARTSINFO g_uart_config_info[] =
{
   {
     CFG_UART1_38400_8N1_20MHZ,          // uint16_t uartConfigNum;   --  Num of the CONFI
     USCI_A1_BASE,                       // uint32_t baseAddress;
     UART_CLOCKSOURCE_SMCLK,             // uint8_t  selectClockSource;
     UART_NO_PARITY,                     // uint8_t  parity;
     UART_LSB_FIRST,                     // uint8_t  msborLsbFirst;
     UART_ONE_STOP_BIT,                  // uint8_t  numberofStopBits;
      32,                                // uint8_t  baudRateReg_BR0;
       0,                                // uint8_t  baudRateReg_BR1;
     145,                                // uint16_t ModReg_MCTL;    -- included overSampling(if any)
     /*UART_USE_RX_INT |*/ UART_USE_TX_INT   // uint16_t configFlags;
   },

   {
      CFG_UART2_38400_8N1_20MHZ,          // uint16_t uartConfigNum;   --  Num of the CONFI
      USCI_A2_BASE,                       // uint32_t baseAddress;
      UART_CLOCKSOURCE_SMCLK,             // uint8_t  selectClockSource;
      UART_NO_PARITY,                     // uint8_t  parity;
      UART_LSB_FIRST,                     // uint8_t  msborLsbFirst;
      UART_ONE_STOP_BIT,                  // uint8_t  numberofStopBits;
      32,                                 // uint8_t  baudRateReg_BR0;
       0,                                 // uint8_t  baudRateReg_BR1;
      145,                                // uint16_t ModReg_MCTL;    -- included overSampling(if any)
      /* UART_USE_RX_INT | */ UART_USE_TX_INT   // uint16_t configFlags;
   },

  //-- Terminator
  {
     0,  // uint16_t uartConfigNum;
     0,  // uint32_t baseAddress;
     0,  // uint8_t  selectClockSource;
     0,  // uint8_t  parity;
     0,  // uint8_t  msborLsbFirst;
     0,  // uint8_t  numberofStopBits;
     0,  // uint8_t  baudRateReg_BR0;
     0,  // uint8_t  baudRateReg_BR1;
     0,  // uint16_t ModReg_MCTL;
     0   // uint16_t configFlags;
  }
};

//--- Global variables for UART

extern BSP_UART_DS * g_p_uart_ds;

//--- Local functions prototypes

static int bsp_sys_uart_set_dma(UARTDS * par);
static int bsp_sys_uart_close(UARTDS * par);
static int bsp_sys_uart_setup(UARTSINFO * par, int uart_config_num);
static void set_uart_rx_timeout(UARTDS * fd, int32_t req, void * arg);
static void set_uart_tx_timeout(UARTDS * fd, int32_t req, void * arg);

static inline void set_io_pins_as_RS485_TX(void)
{
   P5OUT |= (unsigned char)BIT2; //!< New /RE = 1  RX out - 3-State
   P5OUT |= (unsigned char)BIT4; //!<  DE = 1  TX out - Enable
}

static inline void set_io_pins_as_RS485_RX(void)
{
   P5OUT &= ~((unsigned char)BIT2);  //!< New /RE = 0  RX out - Enable
   P5OUT &= ~((unsigned char)BIT4);  //!<  DE = 0  TX out - Disable
}

#if defined USE_UART_0     
static void set_io_uart0_open(void)
{
//  P2.4/P2MAP4             21     UART0_TX              OUT           Debug UART TX (J1/7, TP42)
//  P2.5/P2MAP5             22     UART0_RX              IN/pull-down  Debug UART RX (J1/9, TP87), Interrupt

   P2OUT &= (uint8_t)~BIT5;          //!< RXD - to input
   P2REN |= (uint8_t)BIT5;          //!< Pullup/pulldown resistor enabled  
   P2OUT &= (uint8_t)~BIT5;          //!< 0 = pull-down resistor
    
   P2OUT |= (uint8_t)BIT4;           //!< TXD - to 1
   P2DIR |= (uint8_t)BIT4;           //!< TXD - Output
   P2SEL |= (uint8_t)BIT4 + (uint8_t)BIT5;    //!< TXD & RXD - Peripheral module function is selected for the pin
}                  
#endif

#if defined USE_UART_1     
static void set_io_uart1_open(void)
{
     //!< Set I/O pins as "RS485 RX"

   P5DIR |= (uint8_t) BIT2;
   P5OUT &= (uint8_t)~BIT2;  //!< New /RE = 0  RX out - Enable

   P5DIR |= (uint8_t) BIT4;
   P5OUT &= (uint8_t)~BIT4;  //!<  DE = 0  TX out - Disable

   //!< set UART_A1 Rx/Tx I/O pins

   P8OUT &= (uint8_t)~BIT3;          //!< RXD - to input
   P8REN |= (uint8_t) BIT3;          //!< Pullup/pulldown resistor enabled  
   P8OUT |= (uint8_t) BIT3;          //!< 1= pullup resistor
    
   P8OUT |= (uint8_t)BIT2;           //!< TXD - to 1
   P8DIR |= (uint8_t)BIT2;           //!< TXD - Output
   P8SEL |= (uint8_t)BIT2 + (uint8_t)BIT3;    //!< TXD & RXD - Peripheral module function is selected for the pin
}
#endif

#if defined USE_UART_2     
static void set_io_uart2_open(void)
{
   //!< P9.2   70  OUT    USCI_A2 UART TX   
   //!< P9.3   71  IN     USCI_A2 UART RX   
   //!< P1.6   40  OUT    RTS0    OUT   "0" - active
   //!< P1.5   39  IN     CTS0    In    "0" - TX enable

   P9OUT &= (uint8_t)~BIT3;           //!< RXD - to input
   P9OUT |= (uint8_t) BIT2;           //!< TXD - to 1
   P9DIR |= (uint8_t) BIT2;           //!< TXD - Output
   P9SEL |= (uint8_t) BIT2 + (uint8_t)BIT3;    //!< TXD & RXD - Peripheral module function is selected for the pin

   P1OUT &= (uint8_t)~BIT6;           //!< RTS -> 0
   P1DIR |= (uint8_t) BIT6;

   P1DIR &= (uint8_t)~BIT5;           //!< CTS -> input
}                  
#endif

static void set_io_uart0_close(void)
{
   //!< set I/O pins as regular I/O

   P2SEL &= (uint8_t)(~((uint8_t)BIT4 + (uint8_t)BIT5));

   // Set RX pin(P2.5) - input with pull -down resistor

   P2DIR &= (uint8_t)~BIT5;      
   P2REN  = (uint8_t)BIT5;        //!< Enable pull resistor 
   P2OUT &= (uint8_t)~BIT5;      //!< Here it means BIT5=0 ->pull-down resistor

   // Set TX pin(P2.4) - output with '1' out level

   P2OUT |= (uint8_t)BIT4;      
   P2DIR |= (uint8_t)BIT4;      
}

static inline void set_io_uart1_close(void)
{
   //!< set I/O pins as regular I/O

   P8SEL &= (uint8_t)(~((uint8_t)BIT2 + (uint8_t)BIT3)); 
}         

static inline void set_io_uart2_close(void)
{
   //!< set I/O pins as regular I/O

   P9SEL &= (uint8_t)(~((uint8_t)BIT2 + (uint8_t)BIT3));  

   //!< set CTS (P1.5) as output with level 0

   P1DIR |= (uint8_t)BIT5; 
   P1OUT &= (uint8_t)(~BIT5); 
}         

//============================================================================
// UART 1
//============================================================================

//----------------------------------------------------------------------------
int uart1_tx_buf(char * buf, int len)
{
   int rc;// = TERR_NO_ERR;
   static int fFirst = 0;


   if(buf == NULL || len <= 0)
   {
      rc = -ENOENT;
   }
   else
   {
      if(len > 255)
      {
         len = 255;
      }
      (void)tn_sem_acquire(g_p_uart_ds->g_uart1_ds.semTxBufUart, TN_WAIT_INFINITE);


     //!< Set I/O pins as "RS485 TX"
      set_io_pins_as_RS485_TX();

      if(fFirst == 0)
      {
         fFirst = 1;
         (void)tn_task_sleep(10);
      }
      else
      {
         (void)tn_task_sleep(1);
      }
      rc = bsp_uart_write(&g_p_uart_ds->g_uart1_ds, buf, len);

      (void)tn_task_sleep(1);

     //!< Set I/O pins as "RS485 RX"

      set_io_pins_as_RS485_RX();

      (void)tn_sem_signal(g_p_uart_ds->g_uart1_ds.semTxBufUart);
   }

   return rc;
}

//----------------------------------------------------------------------------
void uart1_tx_char(unsigned char ch)
{
   char buf[4];
   buf[0] = (char)ch;
   
   (void)uart1_tx_buf(buf, 1);
}

//----------------------------------------------------------------------------
int uart1_tx_str(char * str)
{
   int rc = -ENOENT;
   int len;
   
   if(str != NULL) 
   {   
      len = (int)strlen(str);
      rc = uart1_tx_buf(str, len);
   }

   return rc;
}   

//============================================================================
// UART 2
//============================================================================
int uart2_tx_str(char * str)
{
   int rc = -ENOENT;

#ifdef  USE_UART_2
   int len;

   if(str != NULL) 
   {  
      len = (int)strlen(str);
      if(len > 0)
      {
         rc = bsp_uart_write(&g_p_uart_ds->g_uart2_ds, str, len);
      }
   } 
#endif
   return rc;
}

//----------------------------------------------------------------------------
int uart2_tx_buf(char * buf, int len)
{
#ifdef  USE_UART_2
   return bsp_uart_write(&g_p_uart_ds->g_uart2_ds, buf, len);
#else
   return -1;
#endif
}


//----------------------------------------------------------------------------
UARTSINFO * bsp_get_uart_config_info_addr(void)
{
   return (UARTSINFO *)conv_const_ptr_to_ptr((const void*) &g_uart_config_info[0]);
}

//----------------------------------------------------------------------------
UARTDS * bsp_get_uart_data_struct_addr(uint32_t num_uart)
{
   UARTDS * ptr = NULL;
  
#if defined USE_UART_0     
   if(num_uart == (uint32_t)USCI_A0_BASE)
   {
      ptr = &g_p_uart_ds->g_uart0_ds;
   }
   else 
#endif
#if defined USE_UART_1     
   if(num_uart == (uint32_t)USCI_A1_BASE)
   {
      ptr = &g_p_uart_ds->g_uart1_ds;
   }
   else
#endif
   {
#if defined USE_UART_2     
      if(num_uart == (uint32_t)USCI_A2_BASE)
      {
         ptr = &g_p_uart_ds->g_uart2_ds;
      }
#endif
   }

   return ptr;
}

//----------------------------------------------------------------------------
unsigned char * bsp_get_uart_rx_buf(int32_t num_uart)
{
   unsigned char * ptr = NULL;

#if defined USE_UART_0     
   if(num_uart == (int32_t)USCI_A0_BASE)
   {
      ptr = g_p_uart_ds->g_uart0_rx_buf;
   } 
   else 
#endif      

#if defined USE_UART_1     
   if(num_uart == (int32_t)USCI_A1_BASE)
   {
      ptr = g_p_uart_ds->g_uart1_rx_buf;
   }   
   else 
#endif
   {
#if defined USE_UART_2     
      if(num_uart == (int32_t)USCI_A2_BASE)
      {
         ptr = g_p_uart_ds->g_uart2_rx_buf;
      }
#endif      
   }

   return ptr;
}

//----------------------------------------------------------------------------
int bsp_uart_open(UARTDS * fd, char * name, int32_t flags, int32_t mode)
{
   TN_INTSAVE_DATA
     
   UARTSINFO * par;// = NULL;
   UARTDS * pdata  = NULL;
   int rc = TERR_NO_ERR;

   if(fd == NULL)
   {  
      rc = -ENOENT;
   } 
   else
   {  
      if(fd->baseAddress != 0UL)       //!< prevent re-open
      {
         rc = -EEXIST;
      }
      else
      {  
         tn_disable_interrupt();

         if(fd->fd_flags == 0xFFFFFFFFUL) //!<  open is in progress
         {
            rc = -EBUSY;
            tn_enable_interrupt();
         }
         else
         {  
            fd->fd_flags = 0xFFFFFFFFUL;  //!< as flag for open in progress

            tn_enable_interrupt();

            par = bsp_get_uart_config_info_addr();
            while(par->uartConfigNum != 0U)
            {
               if(par->uartConfigNum == (uint16_t)mode)
               {  
                  break;
               }
               par++;
            }

            if(par->uartConfigNum == 0U) // configuration not found
            {  
               rc = -ENOENT;
            }
            else
            {  
               if(par->baseAddress == (uint32_t)USCI_A0_BASE) //!< UART 0
               {
#if defined USE_UART_0     

                  pdata = bsp_get_uart_data_struct_addr(USCI_A0_BASE);

                  if(pdata != NULL)
                  {
                     memset(pdata, 0, sizeof(UARTDS));

                     pdata->baseAddress     = par->baseAddress;
                     pdata->rx_buf          = bsp_get_uart_rx_buf(USCI_A0_BASE);
                     pdata->rx_buf_size     = UART0_RX_BUF_SIZE;
                     pdata->max_tx_buf_size = UART0_TX_MAX_BUFF_SIZE;
                     pdata->semTxUart       = &semTxUart0;
                     pdata->semTxBufUart    = &semTxBufUart0;
                    
                     pdata->rx_timeout      = TN_WAIT_INFINITE;
                     pdata->rx_timeout_cnt  = pdata->rx_timeout;  
                    
                     pdata->tx_timeout      = TN_WAIT_INFINITE;
                     pdata->tx_timeout_cnt  = pdata->tx_timeout;  
                     pdata->use_cts         = FALSE;
                     pdata->rx_cnt          = 0;

                     set_io_uart0_open();

                     UART_op_inc();
                  }  
#endif
               }
               else if(par->baseAddress == (uint32_t)USCI_A1_BASE) //!< UART 1
               {

#if defined USE_UART_1     

                  pdata = bsp_get_uart_data_struct_addr(USCI_A1_BASE);

                  if(pdata != NULL)
                  {
                     (void)memset(pdata, 0, sizeof(UARTDS));
                     
                     pdata->baseAddress     = par->baseAddress;
                     pdata->rx_buf          = bsp_get_uart_rx_buf(USCI_A1_BASE);
                     pdata->rx_buf_size     = UART1_RX_BUF_SIZE;
                     pdata->max_tx_buf_size = UART1_TX_MAX_BUFF_SIZE;
                     pdata->semTxUart       = &semTxUart1;
                     pdata->semTxBufUart    = &semTxBufUart1;
                    
                     pdata->rx_timeout      = TN_WAIT_INFINITE;
                     pdata->rx_timeout_cnt  = pdata->rx_timeout;  
                    
                     pdata->tx_timeout      = TN_WAIT_INFINITE;
                     pdata->tx_timeout_cnt  = pdata->tx_timeout;  
                     pdata->use_cts         = FALSE;
                     pdata->rx_cnt          = 0;

                     set_io_uart1_open();
                  
                     UART_op_inc();
                  }
#endif
               }
               else if(par->baseAddress == (uint32_t)USCI_A2_BASE) //!< UART 2 - GPRS unit
               {

#if defined USE_UART_2     

                  pdata = bsp_get_uart_data_struct_addr(USCI_A2_BASE);

                  if(pdata != NULL)
                  {
                     (void)memset(pdata, 0, sizeof(UARTDS));

                     pdata->baseAddress     = par->baseAddress;
                     pdata->rx_buf          = bsp_get_uart_rx_buf(USCI_A2_BASE);
                     pdata->rx_buf_size     = UART2_RX_BUF_SIZE;
                     pdata->max_tx_buf_size = UART2_TX_MAX_BUFF_SIZE;
                     pdata->semTxUart       = &semTxUart2;
                     pdata->semTxBufUart    = &semTxBufUart2;
                    
                     pdata->rx_timeout      = TN_WAIT_INFINITE;
                     pdata->rx_timeout_cnt  = pdata->rx_timeout;  
                    
                     pdata->tx_timeout      = TN_WAIT_INFINITE;
                     pdata->tx_timeout_cnt  = pdata->tx_timeout;  
                     pdata->use_cts         = FALSE;
                     pdata->rx_cnt          = 0;

                     set_io_uart2_open();

                     UART_op_inc();
                  }
#endif
               } 
               else
               {  
                  rc = -ENODEV;
               }
               
               if(rc != -ENODEV)
               {  
                  (void)bsp_sys_uart_setup(par, (int)mode);

                  rc = bsp_sys_uart_set_dma(pdata);
                  if(rc < 0) //!< error
                  {
                     (void)bsp_sys_uart_close(pdata);
                  }   
               }
            }
         }   
      }  
   }
   
   return rc;
}

//----------------------------------------------------------------------------
int bsp_uart_close(UARTDS * fd)
{
   TN_INTSAVE_DATA
     
   int rc = TERR_NO_ERR; 

   tn_disable_interrupt();

   if(fd == NULL)
   {  
      rc = -ENOENT;
   }
   else
   {  
      if(fd->baseAddress == 0UL)      //!< not open
      {
         rc = -ENXIO;
      }
      else
      {  
         (void)bsp_sys_uart_close(fd);

         fd->baseAddress = 0UL; 
         fd->fd_flags = 0;

         UART_op_dec();
      }   
   }   
   
   tn_enable_interrupt();

   return rc;
}

//----------------------------------------------------------------------------
int bsp_uart_ioctl(UARTDS * fd, int32_t req, void * arg)
{
   int rc = TERR_NO_ERR;
   
   if(fd == NULL)
   {  
      rc = -ENOENT;
   }
   else
   {  
      if(fd->baseAddress != (uint32_t)USCI_A1_BASE)
      {  
         rc = -ENXIO;
      }
      else
      {  
         switch(req) 
         {  
            case (int32_t)SET_UART_RX_TIMEOUT:

               set_uart_rx_timeout(fd, req, arg);

               break;

            case (int32_t)SET_UART_TX_TIMEOUT:

               set_uart_tx_timeout(fd, req, arg);
       
               break;

            default:   
              
               rc = -ENOENT;
               
               break;
         }  
      }
   }   

   return rc;   
}

//----------------------------------------------------------------------------
static int bsp_sys_uart_setup(UARTSINFO * par,  int uart_config_num)
{
   int rc = TERR_NO_ERR; 
   if(par == NULL)
   {  
      rc = -ENOENT;
   }
   else
   {  
       //!< Disable the USCI Module
      HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCSWRST;

       //!< Clock source select
      HWREG8(par->baseAddress + OFS_UCAxCTL1) &= (uint8_t)(~UCSSEL_3);
      HWREG8(par->baseAddress + OFS_UCAxCTL1) |= par->selectClockSource;

       //!< MSB, LSB select
      HWREG8(par->baseAddress + OFS_UCAxCTL0) &= (uint8_t)(~UCMSB);
      HWREG8(par->baseAddress + OFS_UCAxCTL0) |= par->msborLsbFirst;

       //!< UCSPB = 0(1 stop bit) OR 1(2 stop bits)
      HWREG8(par->baseAddress + OFS_UCAxCTL0) &= (uint8_t)(~UCSPB);
      HWREG8(par->baseAddress + OFS_UCAxCTL0) |= par->numberofStopBits;

       //!< Parity
      switch (par->parity)
      {
         case USCI_A_UART_NO_PARITY:
              //!< No Parity
              HWREG8(par->baseAddress + OFS_UCAxCTL0) &= (uint8_t)(~UCPEN);
              break;
         case USCI_A_UART_ODD_PARITY:
              //!< Odd Parity
              HWREG8(par->baseAddress + OFS_UCAxCTL0) |= (uint8_t)UCPEN;
              HWREG8(par->baseAddress + OFS_UCAxCTL0) &= (uint8_t)(~UCPAR);
              break;
         case USCI_A_UART_EVEN_PARITY:
              //!< Even Parity
              HWREG8(par->baseAddress + OFS_UCAxCTL0) |= (uint8_t)UCPEN;
              HWREG8(par->baseAddress + OFS_UCAxCTL0) |= (uint8_t)UCPAR;
              break;
         default:   
              //!< No Parity
              HWREG8(par->baseAddress + OFS_UCAxCTL0) &= (uint8_t)(~UCPEN);
              break;
      }

       //!< Modulation/BaudRate Control Registers

      HWREG8(par->baseAddress + OFS_UCAxBR0)  = par->baudRateReg_BR0;
      HWREG8(par->baseAddress + OFS_UCAxBR1)  = par->baudRateReg_BR1;
      HWREG8(par->baseAddress + OFS_UCAxMCTL) = (uint8_t)par->ModReg_MCTL;


       //!< Asynchronous mode & 8 bit character select & clear mode
      HWREG8(par->baseAddress + OFS_UCAxCTL0) &=  (uint8_t)(~((uint16_t)UCSYNC +
                                                    (uint16_t)UC7BIT +
                                                    (uint16_t)UCMODE_3));

       //!< Configure  UART mode.
      HWREG8(par->baseAddress + OFS_UCAxCTL0) |= (uint8_t)USCI_A_UART_MODE;

       //!< Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
      HWREG8(par->baseAddress + OFS_UCAxCTL1) &= (uint8_t)(~((uint16_t)UCRXEIE 
                                      + (uint16_t)UCBRKIE + (uint16_t)UCDORM +
                                        (uint16_t)UCTXADDR + (uint16_t)UCTXBRK));
      //!< Enable the USCI Module

      HWREG8(par->baseAddress + OFS_UCAxCTL1) &= (uint8_t)(~UCSWRST);
   }
   return rc;
}

//----------------------------------------------------------------------------
static void set_uart_rx_timeout(UARTDS * fd, int32_t req, void * arg)
{
   TN_INTSAVE_DATA
   unsigned long data;

   (void)memcpy((void *)&data, arg, sizeof(long)); 
               
   tn_disable_interrupt();

   fd->rx_timeout = data;

   tn_enable_interrupt();
}

//----------------------------------------------------------------------------
static void set_uart_tx_timeout(UARTDS * fd, int32_t req, void * arg)
{
   TN_INTSAVE_DATA
   unsigned long data;

   (void)memcpy((void *)&data, arg, sizeof(long)); 
               
   tn_disable_interrupt();

   fd->tx_timeout = data;

   tn_enable_interrupt();
}
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
int bsp_uart_write(UARTDS * fd, char * buf, int len) //, int use_sem)
{
   int rc;// = TERR_NO_ERR;
   unsigned short tmp;
 
   if(fd == NULL || buf == NULL || len <= 0)
   {  
      rc = -ENOENT;
   }   
   else
   {  
      if(fd->baseAddress != (uint32_t)USCI_A0_BASE &&
          fd->baseAddress != (uint32_t)USCI_A1_BASE &&
            fd->baseAddress != (uint32_t)USCI_A2_BASE)      //!< not open
      {  
         rc = -ENXIO;
      }
      else
      {  
#if 0
         if(use_sem == TRUE)
         {  
            Semaphore_pend(fd->semTxBufUart, BIOS_WAIT_FOREVER);
         }
#endif
         fd->tx_buf = (unsigned char *)buf;
         fd->tx_idx = 0;
         fd->tx_cnt = MIN(len, fd->max_tx_buf_size);

         if(fd->baseAddress == (uint32_t)USCI_A0_BASE)
         {
            DMACTL2 |= DMA4TSEL_17;  /* DMA channel 4 transfer select 17: USCIA0 transmit */

            __data16_write_addr((unsigned short) &DMA4SA,(unsigned long) fd->tx_buf);  
            tmp = (unsigned short)&DMA4DA;
            __data16_write_addr(tmp,
                                (unsigned long)&UCA0TXBUF); 

            DMA4SZ = (unsigned short)fd->tx_cnt;     // Block size. this counts down to 0

            DMA4CTL = (uint16_t)DMASRCINCR_3 |  // DMA source increment 3: source address incremented
                      (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                      (uint16_t)DMADT_0 |       // DMA transfer mode 0: Single transfer
                      (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

            DMA4CTL |= (uint16_t)DMAEN |        // Start DMA4
                       (uint16_t)DMAIE;         // Enable DMA interrupt
         }
         else if(fd->baseAddress == (uint32_t)USCI_A1_BASE)
         {
            DMACTL0 |=  DMA0TSEL_21;      /* DMA channel 0 transfer select 21: USCIA1 transmit */

            __data16_write_addr((unsigned short) &DMA0SA,(unsigned long) fd->tx_buf);  
            tmp = (unsigned short) &DMA0DA;
            __data16_write_addr(tmp, (unsigned long)&UCA1TXBUF); 

            DMA0SZ  = (unsigned short)fd->tx_cnt;     // Block size. this counts down to 0

            DMA0CTL = (uint16_t)DMASRCINCR_3 |  // DMA source increment 3: source address incremented
                      (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                      (uint16_t)DMADT_0 |       // DMA transfer mode 0: Single transfer
                      (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

            DMA0CTL |= (uint16_t)DMAEN |        // Start DMA0
                       (uint16_t)DMAIE;         // Enable DMA interrupt
         }  
         else
         { 
            if(fd->baseAddress == (uint32_t)USCI_A2_BASE)
            {
               DMACTL1 |=  DMA2TSEL_13;   /* DMA channel 2 transfer select 13: USCIA2 transmit */

               __data16_write_addr((unsigned short) &DMA2SA,(unsigned long) fd->tx_buf);  

               tmp = (unsigned short) &DMA2DA;   
               __data16_write_addr(tmp,
                                   (unsigned long)&UCA2TXBUF); 

               DMA2SZ  = (unsigned short)fd->tx_cnt;      // Block size. this counts down to 0

               DMA2CTL = (uint16_t)DMASRCINCR_3 |  // DMA source increment 3: source address incremented
                         (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                         (uint16_t)DMADT_0 |       // DMA transfer mode 0: Single transfer
                         (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

               DMA2CTL |= (uint16_t)DMAEN |         // Start DMA2
                          (uint16_t)DMAIE;  // Enable DMA interrupt
            }
         }
         rc = tn_sem_acquire(fd->semTxUart, TOUT_1_SEC);  //!< sleep until TX finished

         if(rc == TERR_NO_ERR) //OK
         { 
            rc = fd->tx_cnt;
         }
         else //Timeout
         { 
            rc = -1;
         }

         if(fd->baseAddress == (uint32_t)USCI_A0_BASE)
         {
            DMA4CTL &= (unsigned short)~((unsigned short)DMAEN | (unsigned short)DMAIE);
         }
         else if(fd->baseAddress == (uint32_t)USCI_A1_BASE)
         {
            DMA0CTL &= (unsigned short)~((unsigned short)DMAEN | (unsigned short)DMAIE);
         }  
         else 
         {
            if(fd->baseAddress == (uint32_t)USCI_A2_BASE)
            {    
               DMA2CTL &= (unsigned short)~((unsigned short)DMAEN | (unsigned short)DMAIE);
            }
         }
#if 0
         if(use_sem == TRUE)
         {  
            Semaphore_post(fd->semTxBufUart);
         }
#endif
      }   
   }
   return rc;
}

//----------------------------------------------------------------------------
int bsp_uart_read(UARTDS * fd, char * buf, int len)
{
   int nbytes;
   int rc;//    = -EAGAIN;
   int ind   = 0;
   int fExit = FALSE;
   unsigned short volatile __data16 * sz_reg_addr = NULL;

   if(fd == NULL || buf == NULL || len <= 0)
   {  
      ind = -ENOENT;
   }
   else
   {  
      if(fd->baseAddress == (uint32_t)USCI_A0_BASE)
      {
         sz_reg_addr = &DMA5SZ; 
      }
      if(fd->baseAddress == (uint32_t)USCI_A1_BASE)
      {
         sz_reg_addr = &DMA1SZ; 
      }
      else 
      { 
         if(fd->baseAddress == (uint32_t)USCI_A2_BASE)      //!< not open
         { 
            sz_reg_addr = &DMA3SZ; 
         }
      }

      if(sz_reg_addr == NULL)
      {
         ind = -ENXIO;
      } 
      else
      {  
         nbytes = MIN(len, fd->rx_buf_size);
         fd->rx_timeout_cnt = fd->rx_timeout;  

         while(fExit == FALSE)
         {
      //!< ----- rx char ----------------------------
            if((int)(*sz_reg_addr) + fd->rx_tail != fd->rx_buf_size)
            {
               rc = (int)fd->rx_buf[fd->rx_tail];  //!< rd data
               fd->rx_tail++;
               if(fd->rx_tail >= fd->rx_buf_size)
               {  
                  fd->rx_tail = 0;
               }   
              // fd->rx_cnt--;
            }
            else
            {
               rc = -EAGAIN;
            }

      //---------------------------------

            if(rc >= 0) //!< char, not err
            {
               buf[ind++] = (char)rc;
               if(ind >= nbytes)
               {  
                  fExit = TRUE;
               }   

               fd->rx_timeout_cnt = fd->rx_timeout;  // Rearm timeout
            }
            else //!< now -no char
            {
               if(ind > 0) //!< We received some character(s) before
               {  
                  fExit = TRUE;
               }   
               else // no chars in buf
               {  
                  if(fd->rx_timeout == TN_NO_WAIT)
                  {  
                     ind = -EAGAIN;
                     fExit = TRUE;
                  }   
                  else if(fd->baseAddress != (uint32_t)USCI_A0_BASE &&
                            fd->baseAddress != (uint32_t)USCI_A1_BASE &&
                               fd->baseAddress != (uint32_t)USCI_A2_BASE)      //!< not open
                  { 
                     ind = -ENXIO;
                     fExit = TRUE;
                  }
                  else
                  {  
                     (void)tn_task_sleep(1); //!< Sleep (here ~3.91 mS)

                     if(fd->rx_timeout != TN_WAIT_INFINITE)
                     {
                        fd->rx_timeout_cnt--;
                        if(fd->rx_timeout_cnt == 0UL)
                        {  
                           ind   = -ETIMEDOUT;
                           fExit =  TRUE;
                        } 
                     }
                  }   
               }   
            }
         } //!< while(!fExit)
      }
   }   
   return ind;
}

//----------------------------------------------------------------------------
static int bsp_sys_uart_close(UARTDS * par)
{
   int rc = TERR_NO_ERR;
   
   if(par == NULL)
   { 
      rc = -1;
   }
   else
   {  
      if(par->baseAddress == (uint32_t)USCI_A0_BASE) //!< UART 0
      {
         // Stop DMA4 & DMA 5

         DMA4CTL &= (unsigned short)~((unsigned short)DMAEN | (unsigned short)DMAIE); // Tx     
         DMA5CTL &= (unsigned short)~DMAEN;           // Rx

         //!< Disable RX & TX interrupt -not used but we do it 

         HWREG8(par->baseAddress + OFS_UCAxIE) &= (uint8_t)(~((uint16_t)UCTXIE | (uint16_t)UCRXIE));

         //!< USART - to reset

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCSWRST;

         //!< USART - to dormant(sleep)

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCDORM;

         set_io_uart0_close();

         par->baseAddress = 0UL; //!< Set port as 'not open' 
      }
      else if(par->baseAddress == (uint32_t)USCI_A1_BASE) //!< UART 1
      {

         // Stop DMA0 & DMA 1

         DMA0CTL &= (unsigned short)~((uint16_t)DMAEN | (uint16_t)DMAIE);      
         DMA1CTL &= (unsigned short)~DMAEN;      

         //!< Disable RX & TX interrupt

         HWREG8(par->baseAddress + OFS_UCAxIE) &= (uint8_t)(~((uint16_t)UCTXIE | (uint16_t)UCRXIE));

         //!< USART - to reset

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCSWRST;

         //!< USART - to dormant(sleep)

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCDORM;

         set_io_uart1_close();

         par->baseAddress = 0UL; //!< Set port as 'not open' 
         
      }
      else if(par->baseAddress == (uint32_t)USCI_A2_BASE) //!< UART 2
      {

         // Stop DMA2 & DMA 3

         DMA2CTL &= (unsigned short)~((unsigned short)DMAEN | (unsigned short)DMAIE);      
         DMA3CTL &= (unsigned short)~DMAEN;      

         //!< Disable RX & TX interrupt

         HWREG8(par->baseAddress + OFS_UCAxIE) &= (uint8_t)(~((uint16_t)UCTXIE | (uint16_t)UCRXIE));

         //!< USART - to reset

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCSWRST;

         //!< USART - to dormant(sleep)

         HWREG8(par->baseAddress + OFS_UCAxCTL1) |= (uint8_t)UCDORM;

         set_io_uart2_close();

         par->baseAddress = 0UL; //!< Set port as 'not open' 
         
      }
      else
      {
         rc = -ENODEV;
      }
   }  
 
   return rc;
}

//----------------------------------------------------------------------------
static int bsp_sys_uart_set_dma(UARTDS * par)
{
   int rc = TERR_NO_ERR;
   unsigned short tmp;
 
   if(par == NULL)
   {  
      rc = -ENOENT;
   }
   else
   {  
      DMACTL4 = (uint16_t)DMARMWDIS;

      if(par->baseAddress == (uint32_t)USCI_A0_BASE) //-- UART 0
      {
        //---- DMA 5 - UART0 RX

         DMACTL2 |= DMA5TSEL_16; // DMA channel 5 transfer select 16: USCIA0 receive */

         tmp = (unsigned short)&DMA5SA;
         __data16_write_addr(tmp,
                             (unsigned long)&UCA0RXBUF);  
         __data16_write_addr((unsigned short) &DMA5DA,(unsigned long)par->rx_buf); 

         DMA5SZ  = (unsigned short)par->rx_buf_size;     // Block size. this counts down to 0, then reloads.

         DMA5CTL = (uint16_t)DMADSTINCR_3 |  // DMA destination increment 3: destination address incremented
                   (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                   (uint16_t)DMADT_4 |       // DMA transfer mode 4: Repeated Single transfer
                   (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

         DMA5CTL |= (unsigned short)DMAEN;      // Start DMA1
      }
      else if(par->baseAddress == (uint32_t)USCI_A1_BASE) //-- UART 1
      {
        //---- DMA 1 - UART1 RX

         DMACTL0 |= DMA1TSEL_20; // DMA channel 1 transfer select 20: USCIA1 receive

         tmp = (unsigned short)&DMA1SA;
         __data16_write_addr(tmp,
                             (unsigned long)&UCA1RXBUF);  
         __data16_write_addr((unsigned short) &DMA1DA,(unsigned long)par->rx_buf); 

         DMA1SZ  = (unsigned short)par->rx_buf_size;     // Block size. this counts down to 0, then reloads.

         DMA1CTL = (uint16_t)DMADSTINCR_3 |  // DMA destination increment 3: destination address incremented
                   (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                   (uint16_t)DMADT_4 |       // DMA transfer mode 4: Repeated Single transfer
                   (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

         DMA1CTL |= (unsigned short)DMAEN;      // Start DMA1
      }
      else if(par->baseAddress == (uint32_t)USCI_A2_BASE) //-- UART 2
      {
         //---- DMA 3 - UART2 RX

         DMACTL1 |= DMA3TSEL_12;  // DMA channel 3 transfer select 12: USCIA2 receive 

         tmp = (unsigned short)&DMA3SA;
         __data16_write_addr(tmp, 
                             (unsigned long) &UCA2RXBUF);  
         __data16_write_addr((unsigned short) &DMA3DA,(unsigned long)par->rx_buf); 

         DMA3SZ  = (unsigned short)par->rx_buf_size;     // Block size. this counts down to 0, then reloads.

         DMA3CTL = (uint16_t)DMADSTINCR_3 |  // DMA destination increment 3: destination address incremented
                   (uint16_t)DMASBDB |       // DMA transfer: source byte to destination byte
                   (uint16_t)DMADT_4 |  // DMA transfer mode 4: Repeated Single transfer
                   (uint16_t)DMALEVEL;       // DMA level sensitive trigger select: 1b = Level sensitive (high level)

         DMA3CTL |= (unsigned short)DMAEN;      // Start DMA3
      }
      else
      {
         rc = -ENODEV;
      }
   }

   return rc;
}

//============================================================================
//============================================================================
//============================================================================
//============================================================================
//============================================================================
//============================================================================







