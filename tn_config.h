#ifndef TH_CONFIG_H
#define TH_CONFIG_H

#define  TNKERNEL_PORT_MSP430X       1

   //--- The system configuration (change it for your particular project)


#define  RTOS_TICKS_PER_SECOND     256

#define  TN_STACK_CHECK              1

#define  TN_CHECK_PARAM              1

//#define  TN_MEAS_PERFORMANCE  1

//#define  USE_MUTEXES                 1

//#define  USE_EVENTFLAGS              1

//#define  USE_DYN_OBJ                 1


//#define  TN_TEST                     1 


#define  USER_TIMERS                 1

    //--- User Timers

#if defined USER_TIMERS

#define  TN_USER_TICK_LISTS_CNT        8
#define  TN_USER_TICK_LISTS_MASK       (TN_USER_TICK_LISTS_CNT - 1)

#endif

    //--- OS Timers

#define  TN_OS_TICK_LISTS_CNT        8
#define  TN_OS_TICK_LISTS_MASK       (TN_OS_TICK_LISTS_CNT - 1)

//-- Do not remove or change this line

#define  TN_PRJ_CONFIG               1
#define  TN_RESET_IDLE_STACK         1


#endif
