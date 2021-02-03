#ifndef DATA_EXTERNALS_H_
#define DATA_EXTERNALS_H_

extern BSP_UART_DS * g_p_uart_ds;
extern TN_MAILBOX mailboxSysOp;
extern TN_MAILBOX mailboxUserHiPri;

extern TN_TCB task_SysOp;
extern TN_TCB task_user_hipri;

extern TN_SEM semTxUart1;             //!< UART1 semaphore 
extern TN_SEM semTxUart2;             //!< UART2 semaphore 
extern TN_SEM semTimerLT;
extern TN_SEM semRTC;
extern TN_SEM semCPUFreq;
extern TN_SEM semTestPrint;

extern __no_init char g_test_print_buf[];

extern RTCDATA * g_prtc;
extern LTTMRENTRY g_ltt_timers_arr[MAX_LT_TIMERS];

extern TN_TIMER tmrPreventDS;
extern LEDSDATA g_leds_arr[NUM_LEDS]; 
extern BTNDATA  g_btn_arr[NUM_BUTTONS];
extern DEEPSLEEPDATA g_deep_sleep_data;

#endif /* #ifndef DATA_EXTERNALS_H_ */


