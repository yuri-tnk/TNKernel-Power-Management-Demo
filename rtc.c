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

#ifdef WIN32

#include "stdafx.h"
#include <windows.h>
#include "dt.h"
#include "externals.h"

#else

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

#endif
 

//----- Local functions prototypes ---------

static int bcd2years(unsigned short year);
static int bcd2month(unsigned char month);
static int bcd2dm(unsigned char dm);
static int bcd2hour(unsigned char hour);
static int bcd2min_sec(unsigned char val);

static unsigned char min_sec2bcd(unsigned int val);
static unsigned char hours2bcd(unsigned int hours);
static unsigned char day_of_month2bcd(unsigned int dm);
static unsigned char month2bcd(unsigned int mn);
static unsigned short years2bcd(unsigned int year);
static int rtc_dayofweek(int d, int m, int y);
static int rtc_check_year_month_day(int d, int m, int y);

//----------------------------------------------------------------------------
BOOL is_leap_year(int year)
{  
   BOOL rc = FALSE;
 
   if(((unsigned int)year & 3U) == 0U && ((year % 25) != 0 || 
                                          ((unsigned int)year & 15U) == 0U)) 
   { 
      rc = TRUE;
   }
   return rc;
}

//----------------------------------------------------------------------------
int rtc_init(RTCDATA * prtc, LTT_DT * ref_dt)       
{
   int rc = -1;
   LTT_DT * dt;

   if(prtc != NULL && ref_dt != NULL)
   {   
      prtc->valid_dt = &prtc->rtc_val[0];
      prtc->op_dt    = &prtc->rtc_val[1];
      dt = prtc->valid_dt;

      dt->tm_sec     = ref_dt->tm_sec;  
      dt->tm_min     = ref_dt->tm_min;  
      dt->tm_hour    = ref_dt->tm_hour;
      dt->tm_mday    = ref_dt->tm_mday;
      dt->tm_mon     = ref_dt->tm_mon;
      dt->tm_year    = ref_dt->tm_year;
      dt->tm_wday    = (unsigned int)rtc_dayofweek((int)dt->tm_mday, (int)dt->tm_mon, (int)dt->tm_year);

      rc = do_rtc_set_clock(dt);
   }
   return rc;
}

//----------------------------------------------------------------------------
int rtc_set_clock(LTT_DT * dt)
{
   int rc = -1;

   if(dt != NULL)
   {
      (void)rtc_get_clock(&g_prtc->dt_prev);  // For the timer recreation

      rc = tn_sem_acquire(&semRTC, TOUT_110_MSEC);
      if(rc == TERR_NO_ERR)
      {
         (void)do_rtc_set_clock(dt);

         g_prtc->enable_ltt_check = FALSE;

         rc = ltt_timers_recreate();
         if(rc < 0) // Fatal error, do reset 
         {
               //bsp_force_reset(20, 160);
         }

         g_prtc->enable_ltt_check = TRUE;

         (void)tn_sem_signal(&semRTC);
      }
      else
      {
         rc  = TERR_TIMEOUT;
      }
   }
   
   return rc;
}

//----------------------------------------------------------------------------
int do_rtc_set_clock(LTT_DT * dt)
{
   int rc = -1;
   int dow;
   LTT_DT set_dt;
   LTT_DT * tmp_dt;
   TN_INTSAVE_DATA

//----------------
   (void)memcpy(&set_dt, dt, sizeof(LTT_DT));
   
  //--- Set day of week
   dow = rtc_dayofweek((int)set_dt.tm_mday, (int)set_dt.tm_mon, (int)set_dt.tm_year);
   if(dow >= 0)
   {
      set_dt.tm_wday = (unsigned int)dow & 0x07U; //dt->DayOfWeek;
   //--------------------------------------------------------

      rc = rtc_bin_to_bcd(&set_dt);
      if(rc == TERR_NO_ERR)
      {
#if 0
         while(BAKCTL & LOCKBAK)                //!< Unlock backup system
         {
            BAKCTL &= (unsigned int)(~(LOCKBAK)); 
         }
#endif
         tn_disable_interrupt();
         
         //-- Disable Alarm

         RTCCTL0 &= (unsigned char)(~((unsigned char)RTCAIE | (unsigned char)RTCAIFG)); //-- disable Alarm interrupt
         RTCAMIN  = 0x00;
         RTCAHOUR = 0x00;
         RTCADAY  = 0x00;
         RTCADOW  = 0x00;

         (void)memcpy(g_prtc->op_dt, dt, sizeof(LTT_DT));  // dt in g_prtc->op_dt in  have to be in BCD format   
         g_prtc->op_dt->tm_wday = (unsigned int)dow & 0x07U;        // dt->DayOfWeek;
  
        //-- Configure RTC_B - BCD calendar mode
   
         RTCCTL01 |= (unsigned short)RTCRDYIE | // enable RTC read ready interrupt
                     (unsigned short)RTCBCD |   // BCD mode
                     (unsigned short)RTCHOLD;   // RTC hold 

         RTCSEC  = (unsigned char)set_dt.tm_sec;
         RTCMIN  = (unsigned char)set_dt.tm_min;
         RTCHOUR = (unsigned char)set_dt.tm_hour;
         RTCDOW  = (unsigned char)set_dt.tm_wday;
         RTCDAY  = (unsigned char)set_dt.tm_mday;
         RTCMON  = (unsigned char)set_dt.tm_mon;
         RTCYEAR = (unsigned short)set_dt.tm_year;
 
         //--- Set curr dt

         tmp_dt           = g_prtc->valid_dt;
         g_prtc->valid_dt = g_prtc->op_dt;
         g_prtc->op_dt    = tmp_dt;

         RTCCTL01 &= (unsigned short)(~((unsigned short)RTCHOLD));  // Start RTC calendar mode
            
         tn_enable_interrupt();
      }
      else
      {
         rc = -1;
      }
   }

   return rc;
}

//----------------------------------------------------------------------------
int rtc_get_clock(LTT_DT * dt)  //-- [OUT]
{
   int rc = -1; 
   TN_INTSAVE_DATA
   
   if(dt != NULL)
   {   
      tn_disable_interrupt();
      (void)memcpy(dt, g_prtc->valid_dt, sizeof(LTT_DT));
      tn_enable_interrupt();
       
      rc = TERR_NO_ERR;
   }
   
   return rc;
}

//----------------------------------------------------------------------------
int rtc_get_clock_bcd_async(void)  //-- [OUT]
{
   int rc = TERR_NO_ERR;
   TN_INTSAVE_DATA
  
   while((BAKCTL & (unsigned int)LOCKBAK) != 0U)      //!< Unlock backup system
   {
      BAKCTL &= (unsigned int)(~(LOCKBAK)); 
   }

   while((RTCCTL01 & (unsigned int)RTCRDY) != (unsigned int)RTCRDY) {} // 128/32768 = ~ 3.9 ms Max

   tn_disable_interrupt();

   g_prtc->op_dt->tm_sec  = (unsigned int)RTCSEC;
   g_prtc->op_dt->tm_min  = (unsigned int)RTCMIN; 
   g_prtc->op_dt->tm_hour = (unsigned int)RTCHOUR;
   g_prtc->op_dt->tm_wday = (unsigned int)RTCDOW;
   g_prtc->op_dt->tm_mday = (unsigned int)RTCDAY; 
   g_prtc->op_dt->tm_mon  = (unsigned int)RTCMON;
   g_prtc->op_dt->tm_year = (unsigned int)RTCYEAR;
 
   tn_enable_interrupt();
   
   rtc_update_ptr();

   return rc;
}

//----------------------------------------------------------------------------
int rtc_bin_to_bcd(LTT_DT * dt)
{
   unsigned short rc;
   int res = -1;
  
   if(dt != NULL)
   {   
      rc = min_sec2bcd(dt->tm_sec);
      if(rc != ERR_BYTE) // Not an Err
      {
         dt->tm_sec = rc;
         
         rc = min_sec2bcd(dt->tm_min);
         if(rc != ERR_BYTE) // Not an Err
         {
            dt->tm_min = rc;
         
            rc = hours2bcd(dt->tm_hour);
            if(rc != ERR_BYTE) // Not an Err
            {
               dt->tm_hour = rc;
         
               if(dt->tm_wday <= 6U) // Not an Err
               {
                  rc = day_of_month2bcd(dt->tm_mday);
                  if(rc != ERR_BYTE) // Not an Err
                  {
                     dt->tm_mday = rc;
         
                     rc = month2bcd(dt->tm_mon);
                     if(rc != ERR_BYTE) // Not an Err
                     {
                        dt->tm_mon = rc;
         
                        rc = years2bcd(dt->tm_year);
                        if(rc != ERR_USHORT) // Not en Err
                        {
                           dt->tm_year = rc;
                           res = 0; // Total OK
                        } 
                     }
                  }
               }
            }
         }         
      }
   }
   
   return res;   
}

//----------------------------------------------------------------------------
int rtc_dt_pk_to_bcd(LTT_DT * dst,
                     LTT_DT_PK * src)
{
   unsigned short rc;
   int res = -1;
  
   if(dst != NULL)
   {   
      rc = min_sec2bcd(0U);  //dt->tm_sec);
      if(rc != ERR_BYTE) // Not an Err
      {
         dst->tm_sec = rc;
         
         rc = min_sec2bcd((unsigned int)src->min);
         if(rc != ERR_BYTE) // Not an Err
         {
            dst->tm_min = rc;
         
            rc = hours2bcd((unsigned int)src->hour);
            if(rc != ERR_BYTE) // Not an Err
            {
               dst->tm_hour = rc;
         
               if(dst->tm_wday <= 6U) // Not an Err
               {
                  rc = day_of_month2bcd((unsigned int)src->day_m);
                  if(rc != ERR_BYTE) // Not an Err
                  {
                     dst->tm_mday = rc;
         
                     rc = month2bcd((unsigned int)src->month);
                     if(rc != ERR_BYTE) // Not an Err
                     {
                        dst->tm_mon = rc;
         
                        rc = years2bcd((unsigned int)src->year + (unsigned int)BASE_YEAR);
                        if(rc != ERR_USHORT) // Not en Err
                        {
                           dst->tm_year = rc;
                           res = TERR_NO_ERR; // Total OK
                        } 
                     }
                  }
               }
            }
         }         
      }
   }
   
   return res;   
}

//----------------------------------------------------------------------------
int rtc_bcd_to_bin(LTT_DT * dt)
{
   int rc = -1;

   if(dt != NULL)
   {   
      rc = bcd2min_sec((unsigned char)dt->tm_sec);
      if(rc >= 0)
      {
         dt->tm_sec = (unsigned char)rc;
         
         rc = bcd2min_sec((unsigned char)dt->tm_min);
         if(rc >= 0)
         {
            dt->tm_min = (unsigned char)rc;
         
            rc = bcd2hour((unsigned char)dt->tm_hour);
            if(rc >= 0)
            {
               dt->tm_hour = (unsigned char)rc;
         
               dt->tm_wday &= 0x07U;
               if(dt->tm_wday <= 6U)
               {
                  rc = bcd2dm((unsigned char)dt->tm_mday);
                  if(rc >= 0)
                  {  
                     dt->tm_mday = (unsigned char)rc;
         
                     rc = bcd2month((unsigned char)dt->tm_mon);
                     if(rc >= 0)
                     {
                        dt->tm_mon = (unsigned char)rc;
         
                        rc = bcd2years((unsigned short)dt->tm_year);
                        if(rc >= 0)
                        {
                           dt->tm_year = (unsigned short)rc;
                        }
                     }
                  }
               }
            }
         }     
      }
   }
   
   if(rc > 0)
   {
      rc = TERR_NO_ERR;
   } 
   
   return rc;
}

//----------------------------------------------------------------------------
void rtc_update_ptr(void)
{
   TN_INTSAVE_DATA
   LTT_DT * tmp_dt;
   int rc;

   //-- decode fresh RTC info from BCD to bin
   rc = rtc_bcd_to_bin(g_prtc->op_dt);
   if(rc == TERR_NO_ERR)
   {
      //-- Switch RTC pointers

      tn_disable_interrupt();

      tmp_dt           = g_prtc->valid_dt;
      g_prtc->valid_dt = g_prtc->op_dt;
      g_prtc->op_dt    = tmp_dt;

      tn_enable_interrupt();
   } 
}

//----------------------------------------------------------------------------
static int bcd2years(unsigned short year)
{
   int rc;
   unsigned int year_1000;
   unsigned int year_100;
   unsigned int year_10;
   unsigned int year_1;

   year_1000 = (unsigned int)((year>>12U) & 0x07U);
   year_100  = (unsigned int)((year>>8U) & 0x0FU);
   year_10   = (unsigned int)((year>>4U) & 0x0FU); 
   year_1    = (unsigned int) (year & 0x0FU);

   rc = (int)year_1000 * 1000 + 
        (int)year_100  * 100  +
        (int)year_10   *  10  +
        (int)year_1;

   if(rc < BASE_YEAR || rc > MAX_YEAR) //-- Err
   {   
      rc = -1; 
   }   
   
   return rc;
}

//----------------------------------------------------------------------------
static int bcd2month(unsigned char month)
{
   int rc;
   unsigned int m_hi;
   unsigned int m_lo;

   m_hi = ((unsigned int)month >> 4U) & 1U;
   m_lo = (unsigned int)month & 0x0FU;

   rc = 10 * (int)m_hi + (int)m_lo;
   
   if(rc < 1 || rc > 12) //-- Err
   {   
      rc = -1; 
   }
   
   return rc;
}   

//----------------------------------------------------------------------------
static int bcd2dm(unsigned char dm)
{
   int rc;
   unsigned int dm_hi;
   unsigned int dm_lo;

   dm_hi = (unsigned int)(((unsigned int)dm >> 4U) & 0x3U);
   dm_lo = (unsigned int)((unsigned int)dm & 0x0FU); 

   rc = 10 * (int)dm_hi + (int)dm_lo;
   
   if(rc < 0 || rc > 31)
   {   
      rc = -1; //-- Err
   }
   return rc;
}   

//----------------------------------------------------------------------------
static int bcd2hour(unsigned char hour)
{
   int rc;
   unsigned int hour_hi;
   unsigned int hour_lo;

   hour_hi = (unsigned int)(((unsigned int)hour >> 4U) & 0x3U);
   hour_lo = (unsigned int)((unsigned int)hour & 0x0FU); 

   rc = 10 * (int)hour_hi + (int)hour_lo;
   
   if(rc < 0 || rc > 23)
   {   
      rc = -1; //-- Err
   }
   return rc;
}   

//----------------------------------------------------------------------------
static int bcd2min_sec(unsigned char val)
{
   int rc;
   unsigned int sec_hi;
   unsigned int sec_lo;

   sec_hi = (unsigned int)(((unsigned int)val >> 4U) & 0x7U);
   sec_lo = (unsigned int)((unsigned int)val & 0x0FU); 

   rc = 10 * (int)sec_hi + (int)sec_lo;

   if(rc < 0 || rc > 59) // Err
   {   
      rc = -1; 
   }
   return rc;
}

//----------------------------------------------------------------------------
static unsigned char min_sec2bcd(unsigned int val)
{
   unsigned char rc = ERR_BYTE;
   int i;   

   if(val <= 59U)
   {
      i = 0; 
      while(val >= 10U)
      {
         val -= 10U;
         i++;
      }

      rc = (unsigned char)((unsigned int)i << 4U) | (unsigned char)val;
      rc &= 0x7FU;
   }
   
   return rc; 
}

//----------------------------------------------------------------------------
static unsigned char hours2bcd(unsigned int hours)
{
   unsigned char rc = ERR_BYTE;
   int i;   

   if(hours <= 23U)
   {
      i = 0; 
      while(hours >= 10U)
      {
         hours -= 10U;
         i++;
      }

      rc = (unsigned char)((unsigned int)i<<4U) | (unsigned char)hours;
      rc &= 0x3FU;
   }
   return rc; 
}

//----------------------------------------------------------------------------
static unsigned char day_of_month2bcd(unsigned int dm)
{
   unsigned char rc = ERR_BYTE;
   int i;   

   if(dm <= 31U)
   {
      i = 0; 
      while(dm >= 10U)
      {
         dm -= 10U;
         i++;
      }

      rc = (unsigned char)((unsigned int)i<<4U) | (unsigned char)dm;
      rc &= 0x3FU;
   }
   
   return rc; 
}

//----------------------------------------------------------------------------
static unsigned char month2bcd(unsigned int mn)
{
   unsigned char rc = ERR_BYTE;
   int i;   

   if(mn <= 12U)
   {
      i = 0; 
      while(mn >= 10U)
      {
         mn -= 10U;
         i++;
      }
      rc = (unsigned char)((unsigned int)i<<4U) | (unsigned char)mn;
      rc &= 0x1FU;
   }

   return rc; 
}

//----------------------------------------------------------------------------
static unsigned short years2bcd(unsigned int year)
{
   unsigned short rc = ERR_USHORT;
   unsigned int i;   
   
   if(year >= (unsigned int)BASE_YEAR && year <= (unsigned int)MAX_YEAR)
   {
      i = 0U; 
      while(year >= 1000U)
      {
         year -= 1000U;
         i++;
      }
      rc = (unsigned short)((unsigned int)i<<12U);
      rc &= 0x7FFFU;

      i = 0U; 
      while(year >= 100U)
      {
         year -= 100U;
         i++;
      }
      rc |= (unsigned short)((unsigned int)i<<8U);

      i = 0U; 
      while(year >= 10U)
      {
         year -= 10U;
         i++;
      }
      rc |= (unsigned short)((unsigned int)i<<4U);
      rc |= (unsigned short)year;
   }

   return rc;
}

//----------------------------------------------------------------------------
static int rtc_check_year_month_day(int d, int m, int y)
{
   int rc = 0; // Ok
   int max_day = 28;
   static const short g_dm[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
                               /* Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec */
   
   if(y < BASE_YEAR || y > MAX_YEAR)
   {
      rc = -1;
   }      
   else
   {
      if(m < 1 || m > 12 || d < 1)
      {   
         rc = -1;
      }
      else
      {   
         if(m == 2)
         {
            if(is_leap_year(y) == TRUE)
            {   
               max_day = 29;
            }
            if(d > max_day)
            {   
               rc = -1;
            }
         }
         else
         {
            if(d > g_dm[m-1])
            {   
               rc = -1;
            }
         }
      }
   }
   
   return rc;
}

//----------------------------------------------------------------------------
static int rtc_dayofweek(int d, int m, int y)
{
   int rc; 
   const int t[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

   if(rtc_check_year_month_day(d, m, y) < 0)
   {   
      rc = -1;
   }
   else
   {   
      //y -= m < 3;
      if(m < 3)
      {
         y -= 1;
      } 

      rc = ( y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
   }
   return rc;    
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------




