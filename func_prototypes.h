#ifndef  FUNC_PROTOTYPES_H_
#define  FUNC_PROTOTYPES_H_

   //--- hardware_init.c

void sys_init_cpu(void);

   //--  tn_sprintf.c

int tn_snprintf( char *outStr, int maxLen, const char *fmt, ... );


  //--  rtc.c

void rtc_update_ptr(void);

  //--  deep_sleep.c

void timer_dummy_func(TN_TIMER * tmr, void * param);
void switch_to_op_mode(unsigned int param);
BOOL is_pending_in_mailbox(TN_MAILBOX * mb, TN_TCB * task);
BOOL is_timers_lists_empty(void);
void switch_to_deep_sleep(void);
BOOL check_deep_sleep_cond(void);
void UART_op_inc(void);
void UART_op_dec(void);
int do_set_cpu_freq(int mode);
 
void led_cnt_inc(void);
void led_cnt_dec(void);

  //--  utils.c

void TestPrint(const char *fmt, ...);
void wdt_restart(void);
void wdt_set_long(void);
int get_sem_cnt(TN_SEM * sem);

void stop_XT2_osc(void);

#endif  /* #ifndef  FUNC_PROTOTYPES_H_ */   
