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

extern const int g_month_days[12];
const int g_month_days[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

int ltt_add_minutes(LTT_DT * dt, unsigned long minutes);

int do_bsp_register_timer_lt(int mode,
                             LTT_DT * ltdt,   // reserved - may be NULL
                             unsigned long timeout,
                             void * data, 
                             timer_lt_handler_func timer_callback,
                             int tid);
int do_register_relative(int mode,
                         unsigned long timeout,
                         void * data, 
                         timer_lt_handler_func timer_callback,
                         int tid);
int ltt_compare_dt_tm_pk(LTT_DT_PK * dt, LTT_DT_PK * ref);
int ltt_compare_dt_tm(LTT_DT * dt, LTT_DT * ref);
int ltt_free_entry(LTTMRENTRY * entry);
void set_rtc_alarm(LTT_DT * set_dt);
void stop_alarm_unit(void);
LTTMRENTRY * ltt_timers_alloc_entry(int * timer_id);
int ltt_set_next_timer_to_exec(void);

//----------------------------------------------------------------------------
int ltt_timers_init(void)
{
   LTTMRENTRY * parr;
   int i;

   parr = ltt_timers_get_arr();

   for(i=0; i < MAX_LT_TIMERS + 1; i++)
   {
      (void)memset(parr, 0, sizeof(LTTMRENTRY));
      parr->flags |= LTTL_IS_FREE;

      parr++;
   }

   return TERR_NO_ERR;
}
//----------------------------------------------------------------------------
// This function we call in the thread context
// It is protected by 'semTimerLT' semaphore
//----------------------------------------------------------------------------
long ltt_register_timer(int mode,
                        LTT_DT * ltdt, // reserved, may be NULL
                        unsigned long timeout,
                        void * data, 
                        timer_lt_handler_func timer_callback)
{
   //TN_INTSAVE_DATA

   int rc;
   long res;

   //CS_EXEC(p_g_pwr_info->sem_wait_cnt++;);

   rc = tn_sem_acquire(&semTimerLT, TOUT_110_MSEC);
   if(rc == TERR_NO_ERR)
   {
      res = (int)do_bsp_register_timer_lt(mode,
                                          NULL, 
                                          timeout,
                                          data,
                                          timer_callback,
                                          ADD_TIMER);
      (void)tn_sem_signal(&semTimerLT);
   }
   else
   {
      res = (long)rc;
   }

   //CS_EXEC(p_g_pwr_info->sem_wait_cnt--;);

   //LB3U_LogPrint(MOD_FACILITY, "\r\nRegister Timer LT ID %ld Slot %d \r\n", res, rc);

   return res;
}

//----------------------------------------------------------------------------
// This function we call in the thread context
// It is protected by 'semTimerLT' semaphore
//----------------------------------------------------------------------------
int ltt_unregister_timer(long timer_id)
{
   LTTMRENTRY * parr;
   int rc;// = -1;
   int i;

   parr = ltt_timers_get_arr();

   //CS_EXEC(p_g_pwr_info->sem_wait_cnt++;);

   rc = tn_sem_acquire(&semTimerLT, TOUT_110_MSEC);
   if(rc == TERR_NO_ERR)
   {
      for(i=0; i < MAX_LT_TIMERS; i++)
      {
         if(parr->timer_id == (unsigned long)timer_id)
         {
            if((parr->flags & LTTL_IS_FREE) == LTTL_IS_FREE)
            {
               rc = -3;// ???? > 0   already free - it's error, but not critical
            }
            else // Free(clear) the timer entry 
            {
               (void)memset(parr, 0, sizeof(LTTMRENTRY)); // also set LTTL_IS_FREE to 0
               parr->flags |= LTTL_IS_FREE; //-- Set as free

               //--- Emulator only -------
#ifdef WIN32
               g_ltt_stat.unregistered_cnt++;
#endif
               //-------------------------- 

               rc = ltt_set_next_timer_to_exec();
            }
            break; // Exit the loop any case
         }

         parr++;
      } 

      (void)tn_sem_signal(&semTimerLT);
   }

   //CS_EXEC(p_g_pwr_info->sem_wait_cnt--;);

   return rc;
}

//----------------------------------------------------------------------------
// This function call - in the thread context
// It is protected by 'semTimerLT' semaphore
//----------------------------------------------------------------------------
int ltt_alarm_proc(void * par)
{
   LTTMRENTRY * parr;
   int i;
   int rc; 
   LTT_DT curr_dt;
   LTT_DT tmp_dt;
   LTT_DT_PK curr_dt_pk;
   
#ifdef WIN32
   rtc_get_clock_bcd_async(); 
#endif
   (void)rtc_get_clock(&curr_dt);

   (void)dt_to_pk(&curr_dt,      // src
                  &curr_dt_pk);  // dst

   rc = tn_sem_acquire(&semTimerLT, TOUT_110_MSEC);
   if(rc == TERR_NO_ERR)
   {
      parr = ltt_timers_get_arr();
      for(i=0; i < MAX_LT_TIMERS; i++)
      {
         if(//(parr->flags & LTTL_IS_FREE) == 0 && //!< Item is occuped
              (parr->flags & LTTL_EXEC_NOW) == LTTL_EXEC_NOW) // Have to be exec
         {
            rc = ltt_compare_dt_tm_pk(&parr->exec_dt, // dt 
                                      &curr_dt_pk);      // ref
            if(rc == 0) // To exec
            {
               if((parr->flags & LTT_ONESHOT) == LTT_ONESHOT) //!< timer is one-shot
               {
#ifdef WIN32
                 // Exec emu alarm callback  
                  exec_emu_alarm_callback(i);
#endif

                 // Free (cancel) the timer
                  (void)memset(parr, 0, sizeof(LTTMRENTRY)); // also set LTTL_IS_FREE to 0(occuped)
                   parr->flags |= LTTL_IS_FREE; //-- Set as free
               }         
               else // the timer is is not one-shot
               {   
#ifdef WIN32
               // Exec emu alarm callback  
                  exec_emu_alarm_callback(i);
#endif
                  // set a next(new) exec time     

                  (void)memcpy(&tmp_dt, &curr_dt, sizeof(LTT_DT));

                  (void)ltt_add_minutes(&tmp_dt, parr->timeout);
                  (void)dt_to_pk(&curr_dt,        // src
                                 &parr->base_dt); // dst
                  (void)dt_to_pk(&tmp_dt,         // src
                                 &parr->exec_dt); // dst
               }

              // Exec the timer callback(if any) - any case

               if(parr->callback_func != NULL)
               {  
                  parr->callback_func(parr->timer_id,
                                      curr_dt_pk,
                                      parr->callback_func_data);
               }
            }
            else // The exec time is not equal 'curr_dt'
            {
               if(rc == -1) //  exec_dt < curr_dt (Fatal err - the timer have to be already exec)
               {
                  // Fatal error
#ifdef WIN32
                  printf("Fatal: 1\n"); 
#endif
               }         
            }      
         }
         parr++;
      }      
      
      rc = ltt_set_next_timer_to_exec();
      
      (void)tn_sem_signal(&semTimerLT);
   }

   return rc;
}

//----------------------------------------------------------------------------
// This function call - in the thread context
// It is protected by 'semTimerLT' semaphore
//----------------------------------------------------------------------------
int ltt_timers_recreate(void)
{
   LTTMRENTRY * parr;
   int i;
   int rc;
   
   rc = tn_sem_acquire(&semTimerLT, TOUT_110_MSEC);
   if(rc == TERR_NO_ERR)
   {
    //  rc = -1;
      
      parr = ltt_timers_get_arr();
      for(i=0; i < MAX_LT_TIMERS; i++)
      {
         if((parr->flags & LTTL_IS_FREE) == 0U) //-- It is occuped
         {
            if((parr->flags & LTT_RELATIVE_MODE) == LTT_RELATIVE_MODE) //-- relative mode
            {
               rc = do_register_relative((short)parr->flags,
                                         parr->timeout,
                                         parr->callback_func_data, 
                                         parr->callback_func,
                                         (int)parr->timer_id);
            }
            else //-- Absolute mode
            {
               // Not a part of the example 
               rc = (int)parr->timer_id;
            }
    
            if(rc != (int)parr->timer_id) // There was a regisration error
            {
               //-- kill the timer 
               (void)ltt_free_entry(parr);
               /*rc = ???? */ (void)ltt_set_next_timer_to_exec();
            }
         }
         parr++;
      }

      rc = ltt_set_next_timer_to_exec();
      
      (void)tn_sem_signal(&semTimerLT);
   }
   return rc;
}

//============================================================================
// Routines
//============================================================================

//----------------------------------------------------------------------------
int ltt_set_next_timer_to_exec(void)
{
   LTTMRENTRY * min_timer_ptr = NULL; 
   LTTMRENTRY * parr;
   LTT_DT tmp_dt;
   LTT_DT_PK min_dt;
   LTT_DT_PK curr_dt;

   int i;
   int rc; 
   
   //--- Set max possible date/time
   
   min_dt.day_m = 31UL;
   min_dt.day_w = 6UL;
   min_dt.hour  = 23UL;           
   min_dt.min   = 59UL; 
   min_dt.month = 12UL;
   min_dt.year  = (unsigned long)(MAX_YEAR - BASE_YEAR);
   min_dt.aux   = 0UL;

  //-- Get current time

   (void)rtc_get_clock(&tmp_dt);  
   
   (void)dt_to_pk(&tmp_dt,   // src,
                  &curr_dt); // dst
   //--- find min time to exec - it will be next exec time

   parr = ltt_timers_get_arr();
   for(i=0; i < MAX_LT_TIMERS; i++)
   {
      if((parr->flags & LTTL_IS_FREE) == 0U) // Item is occuped
      {
         //------  a sanity checking --------------------------------
         
         rc = ltt_compare_dt_tm_pk(&parr->exec_dt, // dst 
                                   &curr_dt);      // ref
         if(rc < 0) 
         {
            // Fatal error - timer exec time is less than current time
            // Do reset ???
#ifdef WIN32
             printf("Fatal: 2\n"); 
#endif
         }            

         //------  Find a minimal exec d/t --------------------------
         rc = ltt_compare_dt_tm_pk(&parr->exec_dt, // dst 
                                   &min_dt);       // ref  
         if(rc < 0) // des < ref
         {
            (void)memcpy(&min_dt, &parr->exec_dt, sizeof(LTT_DT_PK)); 
            min_timer_ptr = parr; 
         }
      }
      parr++;
   }

   //------------------------------   
   if(min_timer_ptr == NULL) // Probably, there are no timers to execute
   {
     //-- Stop alarm unit 
     
      stop_alarm_unit();
      rc = -2;
   }
   else
   {   
      min_timer_ptr->flags |= LTTL_EXEC_NOW;
   
      //--- find all entries with equal min abs time

      parr = ltt_timers_get_arr();
      for(i=0; i < MAX_LT_TIMERS; i++)
      {
         if((parr->flags & LTTL_IS_FREE) == 0U) // Item is occuped
         {
            rc = ltt_compare_dt_tm_pk(&parr->exec_dt, // dst 
                                      &min_timer_ptr->exec_dt); // ref
            if(rc == 0) // dst == ref
            {
               parr->flags |= LTTL_EXEC_NOW;
            }
            else //-- Remove exec flag from currently running timer(s) - any case
            {
               parr->flags &= ~LTTL_EXEC_NOW;
            }  
         }
         parr++;
      }

      //--- Load min to timer alarm registers

      rc =  rtc_dt_pk_to_bcd(&tmp_dt,                  // dst
                             &min_timer_ptr->exec_dt); // src
      if(rc == TERR_NO_ERR)
      {   
        // tmp_dt.tm_year += BASE_YEAR;

         set_rtc_alarm(&tmp_dt);
         
         rc = TERR_NO_ERR;
      }
      else
      {
#if defined (WIN32)
         printf("Load Alarm Registers: rtc_dt_pk_to_bcd() failed\n");
#endif
         rc = -1;
      }
   }   
   
   return rc;   
}

//----------------------------------------------------------------------------
LTTMRENTRY * ltt_timers_alloc_entry(int * timer_id)
{
   LTTMRENTRY * parr;
   LTTMRENTRY * res = NULL;
   int i;

   if(timer_id != NULL)
   {   
      parr = ltt_timers_get_arr();
      for(i=0; i < MAX_LT_TIMERS; i++)
      {
         if((parr->flags & LTTL_IS_FREE) == LTTL_IS_FREE)
         {
            (void)memset(parr, 0, sizeof(LTTMRENTRY)); // also set LTTL_IS_FREE to 0
            *timer_id = i + 1; 
            res = parr;
            break;
         }
         parr++;
      }

      if(res == NULL)
      {   
         *timer_id = -1; 
      }   
   }
   
   return res;
}

//----------------------------------------------------------------------------
int ltt_free_entry(LTTMRENTRY * entry)
{
   LTTMRENTRY * parr;
   int i;
   int rc = -1;

   parr = ltt_timers_get_arr();
   for(i=0; i < MAX_LT_TIMERS; i++)
   {
      if(entry == parr)
      {
         if((parr->flags & LTTL_IS_FREE) == LTTL_IS_FREE)
         {
          //-- Already free - error
            rc = -2;
         }
         else
         {
            (void)memset(parr, 0, sizeof(LTTMRENTRY)); // also set LTTL_IS_FREE to 0(occuped)
            parr->flags |= LTTL_IS_FREE; //-- Set as free
            
            rc = TERR_NO_ERR;
         }
         break;
      }
      parr++;
   }
   
   return rc;
}

//----------------------------------------------------------------------------
int do_bsp_register_timer_lt(int mode,
                             LTT_DT * ltdt,   // reserved - may be NULL
                             unsigned long timeout,
                             void * data, 
                             timer_lt_handler_func timer_callback,
                             int tid)
{
   int timer_id = -1;
 
   if(((unsigned int)mode & LTT_RELATIVE_MODE) == LTT_RELATIVE_MODE) //-- relative mode
   {
      timer_id = do_register_relative(mode,
                                      timeout,
                                      data, 
                                      timer_callback,
                                      tid);
   }
   else //-- Absolute mode
   {
      // Not a part of the example 
   }
   
   return timer_id; 
}   


//----------------------------------------------------------------------------
int do_register_relative(int mode,
                         unsigned long timeout,
                         void * data, 
                         timer_lt_handler_func timer_callback,
                         int tid)
{
   LTT_DT curr_dt = {0};
   LTT_DT tmp_dt;
   LTT_DT c_dt; // just for conversion

   LTTMRENTRY * entry = NULL;
   int rc = -1;
   unsigned int reg_flags;// = 0;
   int timer_id = -1;
   unsigned long old_curr_dt;
   unsigned long old_exec_dt;
   unsigned long rem;

   if(timeout == 0UL)
   { 
      goto err_exit;
   }
   (void)rtc_get_clock(&curr_dt);
      
   (void)memcpy(&tmp_dt, &curr_dt, sizeof(LTT_DT));

   if(tid == ADD_NEW_TIMER)  // New Timer
   {
      entry = ltt_timers_alloc_entry(&timer_id);
      if(entry == NULL) //-- No free timers
      {
         //rc =
#ifdef WIN32
         g_ltt_stat.fail_registered_cnt++;
#endif

         goto err_exit;
      }   

      // tmp_dt == curr_dt. We use 'tmp_dt', case we dont want to modify 'curr_dt'

#ifdef WIN32
      g_ltt_stat.registered_cnt++;
      entry->start_tick = g_tick_cnt;
#endif
         
      (void)ltt_add_minutes(&tmp_dt, timeout);
   }
   else // Recreate timer
   {
      timer_id = tid;
      entry = ltt_timers_get_arr();
      entry = &entry[tid - 1]; // case minimal valid tid is 1

      old_curr_dt = conv_tm_to_min(&g_prtc->dt_prev);

      (void)pk_to_dt(&entry->exec_dt, &c_dt);

      old_exec_dt = conv_tm_to_min(&c_dt);
      rem = old_exec_dt - old_curr_dt;

#ifdef WIN32
      g_ltt_stat.recreate_cnt++;
#endif
         
      // tmp_dt == curr_dt. We use 'tmp_dt', case we dont want to modify 'curr_dt'

      (void)ltt_add_minutes(&tmp_dt, rem);  
   }

   if(((unsigned int)mode & LTT_ONESHOT) == LTT_ONESHOT)
   {   
      reg_flags = LTT_RELATIVE_MODE | LTT_ONESHOT; // | PERIODICAL_MONTHLY;
   }
   else
   {   
      reg_flags = LTT_RELATIVE_MODE; // | PERIODICAL_MONTHLY;  
   }
   entry->flags |= reg_flags;

   (void)dt_to_pk(&curr_dt,         // src
                  &entry->base_dt); // dst
   (void)dt_to_pk(&tmp_dt,          // src
                  &entry->exec_dt); // dst

   entry->timeout            = timeout;
   entry->callback_func_data = data; 
   entry->callback_func      = timer_callback;
      
   entry->timer_id           = (unsigned long)timer_id; //-- store timer_id 

   if(tid == ADD_NEW_TIMER)  
   {
      // New Timer - processing the new timer exec time

      rc = ltt_set_next_timer_to_exec();
      if(rc < 0)
      {
         goto err_exit;
      }
   }

   return timer_id; // if OK, it is > 0
   
err_exit:

      // Set entry as free
   if(entry != NULL)
   {      
      (void)memset(entry, 0, sizeof(LTTMRENTRY));
      entry->flags |= LTTL_IS_FREE;
   }
      
   return rc;  // Should be <= 0 here
}   


//----------------------------------------------------------------------------
//  Compiler/endian independed version,
//  0= equal, 1 = dt > ref, -1 = dt < ref
//----------------------------------------------------------------------------
int ltt_compare_dt_tm_pk(LTT_DT_PK * dt, LTT_DT_PK * ref)
{
   int rc;

   if(dt->year <  ref->year) 
   {
      rc = -1;
   }
   else
   {         
      if(dt->year > ref->year)
      {
         rc = 1;
      }
      else
      {   
         if(dt->month < ref->month)
         {
            rc = -1;
         }
         else // equal
         {   
            if(dt->month > ref->month)
            {
               rc = 1;
            }
            else // equal
            {                    
               if(dt->day_m < ref->day_m)
               {
                  rc = -1;
               }
               else
               {                       
                  if(dt->day_m > ref->day_m)
                  {
                     rc = 1;
                  }
                  else  // equal
                  {                        
                     if(dt->hour < ref->hour)
                     {
                       rc = -1;
                     }
                     else
                     {   
                        if(dt->hour > ref->hour)
                        {
                           rc = 1;
                        }
                        else // equal
                        {   
                           if(dt->min < ref->min)
                           {
                              rc = -1;
                           }
                           else
                           {   
                              if(dt->min > ref->min)
                              {
                                 rc = 1;
                              }
                              else // total equal 
                              {
                                 rc = 0;
                              }                                    
                           }
                        }
                     }
                  }
               }
            }
         }
      }         
   }
   
   return rc;
}

//----------------------------------------------------------------------------
//  0= equal, 1 = dt > ref, -1 = dt < ref
//----------------------------------------------------------------------------
int ltt_compare_dt_tm(LTT_DT * dt, LTT_DT * ref)
{
   int rc;

   if(dt->tm_year <  ref->tm_year) 
   {
      rc = -1;
   }
   else
   {         
      if(dt->tm_year > ref->tm_year)
      {
         rc = 1;
      }
      else
      {   
         if(dt->tm_mon < ref->tm_mon)
         {
            rc = -1;
         }
         else // equal
         {   
            if(dt->tm_mon > ref->tm_mon)
            {
               rc = 1;
            }
            else // equal
            {                    
               if(dt->tm_mday < ref->tm_mday)
               {
                  rc = -1;
               }
               else
               {                       
                  if(dt->tm_mday > ref->tm_mday)
                  {
                     rc = 1;
                  }
                  else  // equal
                  {                        
                     if(dt->tm_hour < ref->tm_hour)
                     {
                       rc = -1;
                     }
                     else
                     {   
                        if(dt->tm_hour > ref->tm_hour)
                        {
                           rc = 1;
                        }
                        else // equal
                        {   
                           if(dt->tm_min < ref->tm_min)
                           {
                              rc = -1;
                           }
                           else
                           {   
                              if(dt->tm_min > ref->tm_min)
                              {
                                 rc = 1;
                              }
                              else // total equal 
                              {
                                 rc = 0;
                              }                                    
                           }
                        }
                     }
                  }
               }
            }
         }
      }         
   }
   return rc;
}

//----------------------------------------------------------------------------
int dt_to_pk(LTT_DT * src,
              LTT_DT_PK * dst)
{
   int rc = -1;
   if(src != NULL && dst != NULL)
   {
      dst->min   = (unsigned long)(unsigned short)(src->tm_min  & 0x3FU); // 6; //!< 0..59
      dst->hour  = (unsigned long)(unsigned short)(src->tm_hour & 0x1FU); // 5; //!< 0..23
      dst->day_m = (unsigned long)(unsigned short)(src->tm_mday & 0x1FU); // 5; //!< 1..31 
      dst->day_w = (unsigned long)(unsigned short)(src->tm_wday & 0x7U);  // 3; //!< 0-Sun 6 Sat 
      dst->month = (unsigned long)(unsigned short)(src->tm_mon  & 0xFU);  // 4; //!< 1..12
      if((short)src->tm_year < BASE_YEAR) // check < =100 ????
      {   
         dst->year  = (unsigned long)(unsigned short)(src->tm_year & 0x7FU); // 7; //!< 0-2000 1-2001 ... 100 - 2100   
      }
      else      
      {   
         dst->year  = (unsigned short)((src->tm_year -(unsigned short)BASE_YEAR) & 0x7FU); // 7; //!< 0-2000 1-2001 ... 100 - 2100   
      }
      dst->aux   = 0UL;                  // 2; //!< 0  
   
      rc = TERR_NO_ERR;
   }
   
   return rc;   
}                 

//----------------------------------------------------------------------------
int pk_to_dt(LTT_DT_PK * src,
             LTT_DT * dst)
{
   int rc = -1;
   if(src != NULL && dst != NULL)
   {
      dst->tm_sec = 0U;
      dst->tm_min  = (unsigned short)src->min;   // 6; //!< 0..59
      dst->tm_hour = (unsigned short)src->hour;  // 5; //!< 0..23
      dst->tm_mday = (unsigned short)src->day_m; // 5; //!< 1..31 
      dst->tm_wday = (unsigned short)src->day_w; // 3; //!< 0-Sun 6 Sat 
      dst->tm_mon  = (unsigned short)src->month; // 4; //!< 1..12
      dst->tm_year = (unsigned short)(short)((short)(long)src->year + BASE_YEAR);  // 0-2000 1-2001 ... 100 - 2100   
   
      rc = TERR_NO_ERR;
   }
   
   return rc;   
}                 

//----------------------------------------------------------------------------
unsigned long conv_tm_to_min(LTT_DT * tm)
{
   int i;
   unsigned long res = 0;
   if(tm != NULL)
   {
   //   int16  tm_min;   //<! 0..59
      res += (unsigned long)tm->tm_min;
   //   int16  tm_hour;  //<! 0..23
      res += (unsigned long)tm->tm_hour * 60UL;
   //   int16  tm_mday;  //<! 1..31
      res += (unsigned long)(unsigned short)(tm->tm_mday - 1U) * (unsigned long)DAY_MINUTES;
      for(i = 1; i < (int)tm->tm_mon; i++)
      {
         res += (unsigned long)g_month_days[i - 1] * DAY_MINUTES;
         if(i == 2) 
         {
            if(is_leap_year((int)tm->tm_year) == TRUE)
            {
               res += DAY_MINUTES;
            }
         }
      }

      for(i = BASE_YEAR; i < (int)tm->tm_year; i++)
      {
         res += YEAR_MINUTES;
         if(is_leap_year(i) == TRUE)
         {      
            res += DAY_MINUTES;
         }
      }
   }
   return res;
}

//----------------------------------------------------------------------------
void set_rtc_alarm(LTT_DT * set_dt)
{
   TN_INTSAVE_DATA

   tn_disable_interrupt();

#if defined WIN32    
   RTCADAY  = (((unsigned char)set_dt->tm_mday) & 0x3FU);
   RTCAHOUR = (((unsigned char)set_dt->tm_hour) & 0x3FU);
   RTCAMIN  = (((unsigned char)set_dt->tm_min)  & 0x7FU);
   RTCADOW  = 0;
#else

   RTCADAY  = (unsigned char)(((unsigned char)set_dt->tm_mday) & 0x3FU) | 0x80U;
   RTCAHOUR = (unsigned char)(((unsigned char)set_dt->tm_hour) & 0x3FU) | 0x80U;
   RTCAMIN  = (unsigned char)(((unsigned char)set_dt->tm_min)  & 0x7FU) | 0x80U;
   RTCADOW  = 0;
 
   RTCCTL0 &= (unsigned char)(~RTCAIFG);  //-- Clear Alarm interrupt flag
   RTCCTL0 |= (unsigned char)RTCAIE;     //-- Enable Alarm interrupt
#endif

   tn_enable_interrupt();
}      

//----------------------------------------------------------------------------
void stop_alarm_unit(void)
{      
   TN_INTSAVE_DATA

   tn_disable_interrupt();

#ifndef WIN32
   RTCCTL0 &= (unsigned char)(~((unsigned char)RTCAIE | (unsigned char)RTCAIFG)); //-- disable Alarm interrupt
#endif

   RTCAMIN  = 0x00;
   RTCAHOUR = 0x00;
   RTCADAY  = 0x00;
   RTCADOW  = 0x00;

   tn_enable_interrupt();
}      

//----------------------------------------------------------------------------
LTTMRENTRY * ltt_timers_get_arr(void)
{
   return &g_ltt_timers_arr[0];
}

//----------------------------------------------------------------------------
int ltt_add_minutes(LTT_DT * dt, unsigned long minutes)
{
   int rc;
   unsigned long tmp;

   tmp = conv_tm_to_min(dt);

   tmp  += minutes;

   rc = conv_min_to_tm(tmp,
                       dt); // [OUT]
   return rc;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
/*
* Copyright (C) 2001-2003 by egnite Software GmbH. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. Neither the name of the copyright holders nor the names of
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY EGNITE SOFTWARE GMBH AND CONTRIBUTORS
* ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL EGNITE
* SOFTWARE GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
* THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*
* For additional information see http:www.ethernut.de/
*
* Portions of the following functions are derived from material which is 
* Copyright (c) 1985 by Microsoft Corporation.  All rights are reserved.
*/
//----------------------------------------------------------------------------
int conv_min_to_tm(unsigned long min,
                   LTT_DT * tm) // [OUT]
{
   long in_min = (long)min;
   unsigned long tmp;// = 0; 
   int year = BASE_YEAR; // 2000
   int is_leap;// = 0;
   int tm_yday;
   const int * mdays;                 // pointer to _numdayslp or _numdays

   static const int _lpdays[] = 
   {
       -1, 30, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
   };

   static const int _days[] = 
   {
       -1, 30, 58, 89, 119, 150, 180, 211, 242, 272, 303, 333, 364
   }; 

   while(in_min >= (long)YEAR_MINUTES)
   {
      in_min -= (long)YEAR_MINUTES;
      if(is_leap_year(year) == TRUE)
      {
         if(in_min < (long)DAY_MINUTES) // 365 days for leap year means 31.12 00 min 
         {
            in_min += (long)YEAR_MINUTES; // Recovering the subtracted minutes 
            break;
         }
         else
         {
            in_min -= (long)DAY_MINUTES;
         }
      }
      year++;
   }

   tm->tm_year =  (unsigned short)year; 
   if(is_leap_year(year) == TRUE)
   {
      is_leap = TRUE; // If leap year, set the flag 
   }
   else
   {
      is_leap = FALSE;
   }
   //   Calculate days since January 1st and store it to tm_yday.

   tm_yday = (int) (in_min / (long)DAY_MINUTES);
   in_min -=  (long)((long)tm_yday * (long)DAY_MINUTES);

    //   Determine months since January (Note, range is 0 - 11)
    //   and day of month (range: 1 - 31)

   if(is_leap == TRUE)
   {
      mdays = (const int *)(const void*)_lpdays;
   }
   else
   {
      mdays = (const int *)(const void*)_days;
   }

   for(tmp = 1; mdays[tmp] < tm_yday; tmp++){}

   tm->tm_mon  = (unsigned short)tmp; // Range - 1..12 
   tm->tm_mday = (unsigned short)(short)(tm_yday - mdays[tmp - 1UL]);

    // Calculate the time of day from the remaining minutes 

   tm->tm_hour = (unsigned short)(short)(in_min / 60L);
   in_min -= (long)(unsigned long)((unsigned long) tm->tm_hour * 60UL);

   tm->tm_min = (unsigned short)(short)in_min;

   //-- Unsupported fields

   tm->tm_sec   = 0; //<! Reserved here
   tm->tm_wday  = 0; //<! 0-Sunday,6-Saturday
  // tm->tm_isdst = 0; //<! Reserved

   return 0; // OK
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
