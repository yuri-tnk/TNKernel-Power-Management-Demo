#ifndef  PRJ_CONF_H
#define  PRJ_CONF_H

#define  TERR_CANCEL_OP               (-189)



#define TEST_MEM_POOL_SIZE   4096

// In the project, we use UART1  only as debug console
 
//#define  USE_UART_0   1
#define  USE_UART_1   1
//#define  USE_UART_2   1

#define  NUM_LEDS               4 
#define  NUM_BUTTONS            2  /**< total number of buttons*/
#define  MAX_LT_TIMERS          8  /**< Max Number of LT timers in the system   */ 

  // In the project, we use 2 LEDs
#define  LED_ID_1               1
#define  LED_ID_2               2

#endif


