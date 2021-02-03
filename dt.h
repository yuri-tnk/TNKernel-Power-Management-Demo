#ifndef DT_H_
#define DT_H_

#define DAY_MINUTES           (24UL * 60UL)
#define YEAR_MINUTES          (unsigned long)(365UL * DAY_MINUTES)

#ifdef WIN32

#define TERR_NO_ERR       0
#define TERR_TIMEOUT   -110
#define TN_INTSAVE_DATA
#define TOUT_110_MSEC     110

#endif

#define ADD_NEW_TIMER   (-1)
#define ERR_BYTE        0xFFU
#define ERR_USHORT      0xFFFFU

typedef struct _LTT_DT
{
   unsigned short  tm_sec;   //<! Reserved here
   unsigned short  tm_min;   //<! 0..59
   unsigned short  tm_hour;  //<! 0..23
   unsigned short  tm_mday;  //<! 1..31
   unsigned short  tm_mon;   //<! 1..12
   unsigned short  tm_year;  //<! 0-2000, 1-2001 etc.
   unsigned short  tm_wday;  //<! 0-Sunday,6-Saturday - not used here
}LTT_DT;

typedef struct _LTT_DT_PK  //!< size 32 bits
{
   unsigned long min:   6; //!< 0..59
   unsigned long hour:  5; //!< 0..23
   unsigned long day_m: 5; //!< 1..31 
   unsigned long day_w: 3; //!< 0-Sun 6 Sat 
   unsigned long month: 4; //!< 1..12
   unsigned long year:  7; //!< 0-2000 1-2001 ... 100 - 2100   
   unsigned long aux:   2; //!< 0  
}LTT_DT_PK; 


/**
 *
 * This is a global RTC data structure(single instance for the project)
 */
typedef struct _RTCDATA  
{
  LTT_DT rtc_val[2];        //!< Two instances for the Ping-Pong operating
  LTT_DT * valid_dt;        //!< Pointer to valid data/time info structure
  LTT_DT * op_dt;           //!< Pointer to data/time info structure to fill
  int enable_ltt_check;    //!< For lt timers recreation
  LTT_DT dt_prev;           //!< For lt timers recreation  
}RTCDATA;


/**
 *  Long Term Timer operating mode 
 */
#define  LTT_REGULARE_MODE     16U  //!< long term timer operating mode 
#define  LTT_WEEK_MODE         32U  //!< long term timer operating mode 
#define  LTT_RELATIVE_MODE     64U  //!< long term timer operating mode 
#define  LTT_ONESHOT          128U  //!< long term timer operating mode 

#define  LTTL_EXEC_NOW        512U  //!< long term timer flag - exec mode flag (internal)
#define  LTTL_IS_FREE        1024U  //!< long term timer flag - slot is free/occuped (internal)

// *                     Config

#define  BASE_YEAR           2000   /**< Minimal(base) year for RTC operations  */
#define  MAX_YEAR            2100   /**< Maximal year for RTC operations  */

//------------------------------------------

/**< Mode to distinguish between create or re-create timer options */  
#define  ADD_TIMER           (-1)


/**
 *
 *  the prorotype for the lt timer callback function
 */
typedef int (* timer_lt_handler_func) (int timer_id, LTT_DT_PK lt_timestamp, void * data);

/**
 *
 *  this is entry in the lt timers table
 */
typedef struct _LTTMRENTRY   //-- 24 bytes
{
   LTT_DT_PK base_dt; //!< the base timer date/time
   LTT_DT_PK exec_dt; //!< the timer current(next) expiring data/time

   unsigned long timer_id;               //!< timer ID
   unsigned long timeout;                //!< timer timeout(for LTT_REGULARE_MODE only)
   unsigned int flags;                   //!< for internal operating
   timer_lt_handler_func callback_func;  //!< timer callback function      - may be NULL
   void * callback_func_data;            //!< timer callback function data - may be NULL

#ifdef WIN32
   int start_tick;
#endif
}LTTMRENTRY;

//----------------------------------------------------------
#ifdef WIN32 
typedef struct _TN_SEM
{
  HANDLE hSem;
}TN_SEM;

int tn_sem_create(TN_SEM * sem, int init_count);
int tn_sem_acquire(TN_SEM * sem, long timeout);
int tn_sem_signal(TN_SEM * sem);
#endif

BOOL is_leap_year(int year);

unsigned long conv_tm_to_min(LTT_DT * tm);
int conv_min_to_tm(unsigned long min,
                   LTT_DT * tm); // [OUT]

int get_max_day(int month, int year);
int exec_emu_alarm_callback(int idx);

int emu_run_rtc(int * res); 
int ltt_alarm_proc(void * par);

#if 0
typedef struct _TESTARRITEM
{
   BOOL alarm_event;
   unsigned long timeout;
   LTT_DT_PK base_dt; //!< the base timer date/time
   LTT_DT_PK exec_dt; //!< the timer current(next) expiring data/time
   struct tm curr_dt;

   int ticks;

}TESTARRITEM;
#endif

typedef struct _LTTSTAT
{
   int registered_cnt;
   int fail_registered_cnt;
   int recreate_cnt;
   int unregistered_cnt;
   int not_free_cnt;
   int exp_cnt;
   int exp_ok_cnt;
   int exp_fail_cnt;
   int time_change_cnt;
}LTTSTAT;

LTTMRENTRY * ltt_timers_get_arr(void);

void check_and_dump_expiring_result(int idx);
int dt_to_pk(LTT_DT * src,
              LTT_DT_PK * dst);
int pk_to_dt(LTT_DT_PK * src,
             LTT_DT * dst);
int ltt_timers_init(void);
int ltt_timers_recreate(void);
int ltt_unregister_timer(long timer_id);

int rtc_init(RTCDATA * prtc, LTT_DT * ref_dt);
int rtc_set_clock(LTT_DT * dt);
int do_rtc_set_clock(LTT_DT * dt);
int rtc_get_clock(LTT_DT * dt);
int rtc_bin_to_bcd(LTT_DT * dt);
int rtc_dt_pk_to_bcd(LTT_DT * dst,
                     LTT_DT_PK * src);
int rtc_bcd_to_bin(LTT_DT * dt);

long ltt_register_timer(int mode,
                        LTT_DT * ltdt, // reserved, may be NULL
                        unsigned long timeout,
                        void * data, 
                        timer_lt_handler_func timer_callback);
int rtc_get_clock_bcd_async(void); 
void change_emu_time(int min_to_add);
void dump_stat(void);
void calc_not_free_timer(void);

#define  TOUT_110_MSEC   28U
#define  TOUT_200_MSEC   51U
#define  TOUT_1_SEC      256UL 
#define  TOUT_5_SEC      (5UL * 256UL)
#define  TOUT_10_SEC     (10UL * 256UL) 
#define  TOUT_20_SEC     (20UL * 256UL) 


#endif

