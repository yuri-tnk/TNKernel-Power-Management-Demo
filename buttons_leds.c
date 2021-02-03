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

static void bsp_btn_init(BTNDATA * bi,
                         unsigned short port_addr,
                         unsigned int pin,
                         int num,
                         int on_level,
                         send_btn_state_func send_btn_state);
static BTNDATA * get_btn_arr_ptr(void);
static void btn_proc(BTNDATA * bi);
static void btn_is_released_proc(BTNDATA * bi);
static void btn_is_pressed_proc(BTNDATA * bi);
static int send_btn_state(int btn, int state);

static void led_add(LEDSDATA * ld,
                    int led_id,
                    int on_level, 
                    unsigned short port_addr,
                    unsigned int pin);

static void  led_off(LEDSDATA * ld);
static void  led_on(LEDSDATA * ld);

//----------------------------------------------------------------------------
static int send_btn_state(int btn, int state)
{
   SYS_MSG msg;
   int rc = TERR_ILUSE;

   msg.op_code = 0; 

   if(btn == BTN_ID_1)
   {
      if(state == BTN_STATE_BTN_PRESSED)
      {
         msg.op_code = SYS_MSG_BTN1_PRESSED;
      }
      else if(state == BTN_STATE_BTN_RELEASED)
      {
         msg.op_code = SYS_MSG_BTN1_RELEASED;
      }
   }

#if 0
   else if(btn == BTN_ID_2)
   {
      if(state == BTN_STATE_BTN_PRESSED)
      {
         msg.data = (void*)((long)MSG_BTN1_PRESSED);
      }
      else if(state == BTN_STATE_BTN_RELEASED)
      {
         msg.data = (void*)((long)MSG_BTN1_RELEASED);
      }
   }
#endif

   if(msg.op_code != 0)
   { 
      rc = tn_mailbox_send(&mailboxSysOp,
                           &msg,
                           sizeof(SYS_MSG),
                           TN_NO_WAIT); 
   }
   return rc;
}

//----------------------------------------------------------------------------
static BTNDATA * get_btn_arr_ptr(void)
{
   return &g_btn_arr[0];
}

//----------------------------------------------------------------------------
void button_processing(void)
{
   BTNDATA * bi;
   int i;
   
   bi = get_btn_arr_ptr();
      
   for(i = 0; i < NUM_BUTTONS; i++)
   {  
       btn_proc(bi);
       bi++;
   }
}

//----------------------------------------------------------------------------
void buttons_init(void)
{
   BTNDATA * bi;
   
   bi = get_btn_arr_ptr();
   
   //!< SW2 P2.1 BTN_ID_1
   
   bsp_btn_init(bi,
                BTN_PORT_ADDR,         //!< port_addr
                BIT1,                  //!< pin
                BTN_ID_1,        //!< num
                BTN_IS_PRESSED_AT_0,
                (send_btn_state_func)send_btn_state);
   bi++;      
   
#if 0
   //!< SW1 P2.0 BTN_ID_2
  
   bsp_btn_init(bi,
                BTN_PORT_ADDR,         //!< port_addr
                BIT0,                  //!< pin
                BTN_ID_2,     //!< num
                BTN_IS_PRESSED_AT_0,
                (send_btn_state_func)send_btn_state);
#endif

}

//----------------------------------------------------------------------------
static void btn_is_pressed_proc(BTNDATA * bi)
{
   //!< Button is pressed
   
   if(bi->bounce_cnt > 0)
   {  
      bi->bounce_cnt--;
      if(bi->bounce_cnt == 0)
      {
         if(bi->status != BTN_STATE_BTN_UNKNOWN)
         { 
            if(bi->send_btn_state_handler != NULL)
            { 
               bi->send_btn_state_handler(bi->num, BTN_STATE_BTN_PRESSED);
            }  
         }   
         bi->status = BTN_STATE_BTN_ON;
      }
   }   
}

//----------------------------------------------------------------------------
static void btn_is_released_proc(BTNDATA * bi)
{
   //!< Button is released
  
   if(bi->bounce_cnt < bi->max_bounce_cnt)
   {
      bi->bounce_cnt++;
      if(bi->bounce_cnt == bi->max_bounce_cnt)
      {
         if(bi->status != BTN_STATE_BTN_UNKNOWN)
         { 
            if(bi->send_btn_state_handler != NULL)
            { 
               bi->send_btn_state_handler(bi->num, BTN_STATE_BTN_RELEASED);
            } 
         }   
         bi->status = BTN_STATE_BTN_OFF;
      } 
   }
}

//----------------------------------------------------------------------------
static void btn_proc(BTNDATA * bi)
{
   volatile unsigned char port_val;

   port_val = *bi->port;
   if((port_val & bi->pin) == 0) //!< Button is pressed
   {
      if(bi->on_level == BTN_IS_PRESSED_AT_0)
      {
         btn_is_pressed_proc(bi);
      }
      else if(bi->on_level == BTN_IS_PRESSED_AT_1)
      {
         btn_is_released_proc(bi);
      }
   }
   else
   {
      if(bi->on_level == BTN_IS_PRESSED_AT_0)
      {
         btn_is_released_proc(bi);
      }
      else if(bi->on_level == BTN_IS_PRESSED_AT_1)
      {
         btn_is_pressed_proc(bi);
      }
   }
}

//----------------------------------------------------------------------------
static void bsp_btn_init(BTNDATA * bi,
                         unsigned short port_addr,
                         unsigned int pin,
                         int num,
                         int on_level,
                         send_btn_state_func send_btn_state)
{
  
   bi->bounce_cnt     = MAX_BTN_BOUNCE_SUPPR_DLY >> 1;
   bi->max_bounce_cnt = MAX_BTN_BOUNCE_SUPPR_DLY;
   
   bi->port = (unsigned char *) port_addr;
   bi->pin  = pin;
   bi->num  = num;
   bi->on_level = on_level;
     
   bi->status = BTN_STATE_BTN_UNKNOWN;
   bi->send_btn_state_handler = send_btn_state;
}

//----------------------------------------------------------------------------
LEDSDATA * get_leds_arr(void)
{
   return &g_leds_arr[0]; 
}

//----------------------------------------------------------------------------
void leds_init(void)
{
   LEDSDATA * ld = get_leds_arr();

   memset(ld, 0, sizeof(LEDSDATA) * NUM_LEDS);

   led_add(ld,
           LED_ID_1,     // int led_id,
           1,            // int on_level, 
           LED_PORT_ADDR,        // unsigned short port_addr,
           BIT6);        // unsigned int pin)

   led_add(ld,
           LED_ID_2,     // int led_id,
           1,             // int on_level, 
           LED_PORT_ADDR,        // unsigned short port_addr,
           BIT0);         // unsigned int pin)
}

//----------------------------------------------------------------------------
void set_led_on(int led_id)
{
   TN_INTSAVE_DATA
   LEDSDATA * ld = get_leds_arr();
   int i;

   for(i = 0; i < NUM_LEDS; i++)
   {
      if(ld->led_id == led_id)
      {
         tn_disable_interrupt();  
         ld->led_state = LED_STATE_ON; 
         tn_enable_interrupt();  

         led_on(ld);    

         break;
      }
      ld++;       
   }
}

//----------------------------------------------------------------------------
void set_led_off(int led_id)
{
   TN_INTSAVE_DATA
   LEDSDATA * ld = get_leds_arr();
   int i;

   for(i = 0; i < NUM_LEDS; i++)
   {
      if(ld->led_id == led_id)
      {
         tn_disable_interrupt();  
         ld->led_state = LED_STATE_OFF; 
         tn_enable_interrupt();  

         led_off(ld);  
 
         break;
      }
      ld++;       
   }
}

//----------------------------------------------------------------------------
void set_led_blink(int led_id,
                   int blink_start_state,
                   unsigned int blink_period, 
                   unsigned int blink_times)
{
   //TN_INTSAVE_DATA
   LEDSDATA * ld = get_leds_arr();
   int i;

   for(i = 0; i < NUM_LEDS; i++)
   {
      if(ld->led_id == led_id)
      {
         led_cnt_inc(); // Prevent deep sleep

        // tn_disable_interrupt();

         ld->blink_start_state = blink_start_state;

         if(blink_start_state == LED_ON_AT_BLINK_START)
         {  
            ld->led_state = LED_STATE_ON_BLINK;
            led_on(ld);
         }
         else
         {
            ld->led_state = LED_STATE_OFF_BLINK;
            led_off(ld);
         }
 
         ld->blink_period     = blink_period;
         if(ld->blink_period < 2U)
         {
            ld->blink_period = 2U;   
         }

         ld->blink_times      = blink_times;
         if(ld->blink_times < 1U)
         {
            ld->blink_times = 1U;   
         }

         ld->blink_period_cnt = ld->blink_period >> 1U;
         ld->blink_times_cnt  = ld->blink_times;

    //     tn_enable_interrupt();

         break;
      }
      ld++;       
   }
}

//----------------------------------------------------------------------------
static void led_add(LEDSDATA * ld,
                    int led_id,
                    int on_level, 
                    unsigned short port_addr,
                    unsigned int pin)
{
   int i;

   for(i = 0; i < NUM_LEDS; i++)
   {
      if(ld->led_id == 0)  // Empty slot
      {
         ld->led_id    = led_id;
         ld->port_addr = port_addr;
         ld->pin       = pin;
         ld->on_level  = on_level;

         led_off(ld); 

         break;
      }
      ld++;       
   }
}

//----------------------------------------------------------------------------
void sys_led_processing(void)
{
   LEDSDATA * ld = get_leds_arr();
   int i;

   for(i = 0; i < NUM_LEDS; i++)
   {
      if(ld->led_id != 0)  // Not empty slot
      {
         if(ld->led_state == LED_STATE_OFF_BLINK ||
              ld->led_state == LED_STATE_ON_BLINK) 
         {
            ld->blink_period_cnt--;
            if(ld->blink_period_cnt == 0)
            {   
               ld->blink_period_cnt = ld->blink_period >> 1;

               if(ld->led_state == LED_STATE_ON_BLINK)
               {
                  if(ld->blink_start_state != LED_ON_AT_BLINK_START)
                  {
                     if(ld->blink_times != BLINK_INFINITE)
                     {
                        ld->blink_times_cnt--; 
                        if(ld->blink_times_cnt == 0)
                        { 
                           // Stop blink  
                           ld->led_state = LED_STATE_ON;
                           led_cnt_dec();
                        }  
                     } 
                  } 
                  if(ld->led_state != LED_STATE_ON)
                  {
                     led_off(ld);
                     ld->led_state = LED_STATE_OFF_BLINK; 
                  } 
               }
               else // if(ld->led_state == LED_STATE_OFF_BLINK)
               {
                  if(ld->blink_start_state == LED_ON_AT_BLINK_START)
                  {
                     if(ld->blink_times != BLINK_INFINITE)
                     {
                        ld->blink_times_cnt--; 
                        if(ld->blink_times_cnt == 0)
                        { 
                           // Stop blink  
                           ld->led_state = LED_STATE_OFF;
                           led_cnt_dec();
                        }  
                     }
                  }
 
                  if(ld->led_state != LED_STATE_OFF)
                  {
                     led_on(ld);
                     ld->led_state = LED_STATE_ON_BLINK; 
                  } 
               }
            }
         }
      }
      ld++;       
   }
}

//----------------------------------------------------------------------------
static void  led_off(LEDSDATA * ld)
{ 
   unsigned char * ptr = (unsigned char *)ld->port_addr;
 
  // Set led to off
   if(ld->on_level > 0) // on - by 1
   {
      *ptr &= ~ld->pin;
   }  
   else
   {
      *ptr |= ld->pin;
   }
}

//----------------------------------------------------------------------------
static void  led_on(LEDSDATA * ld)
{ 
   unsigned char * ptr = (unsigned char *)ld->port_addr;

  // Set led to on
   if(ld->on_level > 0) // on - by 1
   {
      *ptr |= ld->pin;
   }  
   else
   {
      *ptr &= ~ld->pin;
   }
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

