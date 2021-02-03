#ifndef  TYPES_H
#define  TYPES_H



#ifndef  MIN
#define  MIN(a,b)    (((a)<(b))?(a):(b))
#endif

#ifndef  MAX
#define  MAX(a,b)    (((a)>(b))?(a):(b))
#endif

#define  EEXIST      (100)
#define  EBUSY       (101)
#define  ENOENT      (102)
#define  ENODEV      (103)
#define  ENXIO       (104)
#define  EAGAIN      (105)
#define  ETIMEDOUT   (110)


#define  EVT_EXEC_SHELL    1 


//!< This is a part of the standard MSP430 CPU definitions
  
#define STATUS_SUCCESS  0x01   /**< op ok */
#define STATUS_FAIL     0x00   /**< op failed */

  
//!< Access to H/W registers
  
#define HWREG8(x) \
              (*((volatile uint8_t*)((uint16_t)x)))     /**<  8 bit access */ 
#define HWREG16(x) \
              (*((volatile uint16_t*)((uint16_t)x)))    /**< 16 bit access */ 
#define HWREG32(x) \
              (*((volatile uint32_t*)((uint16_t)x)))    /**< 32 bit access */ 
                
/**
* PMM - Power Management System
*/
                
#ifdef  __MSP430_HAS_PMM__            /**< Definition to show that Module is available */

#define OFS_PMMCTL0           (0x0000u)  /**< PMM Control 0 */
#define OFS_PMMCTL0_L          OFS_PMMCTL0
#define OFS_PMMCTL0_H          OFS_PMMCTL0+1U
#define OFS_PMMCTL1           (0x0002u)  /**< PMM Control 1 */
#define OFS_PMMCTL1_L          OFS_PMMCTL1
#define OFS_PMMCTL1_H          OFS_PMMCTL1+1U
#define OFS_SVSMHCTL          (0x0004u)  /**< SVS and SVM high side control register */
#define OFS_SVSMHCTL_L         OFS_SVSMHCTL
#define OFS_SVSMHCTL_H         OFS_SVSMHCTL+1U
#define OFS_SVSMLCTL          (0x0006u)  /**< SVS and SVM low side control register */
#define OFS_SVSMLCTL_L         OFS_SVSMLCTL
#define OFS_SVSMLCTL_H         OFS_SVSMLCTL+1U
#define OFS_SVSMIO            (0x0008u)  /**< SVSIN and SVSOUT control register */
#define OFS_SVSMIO_L           OFS_SVSMIO
#define OFS_SVSMIO_H           OFS_SVSMIO+1U
#define OFS_PMMIFG            (0x000Cu)  /**< PMM Interrupt Flag */
#define OFS_PMMIFG_L           OFS_PMMIFG
#define OFS_PMMIFG_H           OFS_PMMIFG+1U
#define OFS_PMMRIE            (0x000Eu)  /**< PMM and RESET Interrupt Enable */
#define OFS_PMMRIE_L           OFS_PMMRIE
#define OFS_PMMRIE_H           OFS_PMMRIE+1U

#define PMMPW               (0xA500u)  /**< PMM Register Write Password */
#define PMMPW_H             (0xA5)     /**< PMM Register Write Password for high word access */

/** 
*   PMMCTL0 Control Bits 
*/
                
#define PMMCOREV0           (0x0001u)  /**< PMM Core Voltage Bit: 0 */
#define PMMCOREV1           (0x0002u)  /**< PMM Core Voltage Bit: 1 */
#define PMMSWBOR            (0x0004u)  /**< PMM Software BOR */
#define PMMSWPOR            (0x0008u)  /**< PMM Software POR */
#define PMMREGOFF           (0x0010u)  /**< PMM Turn Regulator off */
#define PMMHPMRE            (0x0080u)  /**< PMM Global High Power Module Request Enable */

/**< PMMCTL0 Control Bits */
#define PMMCOREV0_L         (0x0001u)  /**< PMM Core Voltage Bit: 0 */
#define PMMCOREV1_L         (0x0002u)  /**< PMM Core Voltage Bit: 1 */
#define PMMSWBOR_L          (0x0004u)  /**< PMM Software BOR */
#define PMMSWPOR_L          (0x0008u)  /**< PMM Software POR */
#define PMMREGOFF_L         (0x0010u)  /**< PMM Turn Regulator off */
#define PMMHPMRE_L          (0x0080u)  /**< PMM Global High Power Module Request Enable */

#define PMMCOREV_0          (0x0000u)  /**< PMM Core Voltage 0 (1.35V) */
#define PMMCOREV_1          (0x0001u)  /**< PMM Core Voltage 1 (1.55V) */
#define PMMCOREV_2          (0x0002u)  /**< PMM Core Voltage 2 (1.75V) */
#define PMMCOREV_3          (0x0003u)  /**< PMM Core Voltage 3 (1.85V) */

/**< PMMCTL1 Control Bits */
#define PMMREFMD            (0x0001u)  /**< PMM Reference Mode */
#define PMMCMD0             (0x0010u)  /**< PMM Voltage Regulator Current Mode Bit: 0 */
#define PMMCMD1             (0x0020u)  /**< PMM Voltage Regulator Current Mode Bit: 1 */

/**< PMMCTL1 Control Bits */
#define PMMREFMD_L          (0x0001u)  /**< PMM Reference Mode */
#define PMMCMD0_L           (0x0010u)  /**< PMM Voltage Regulator Current Mode Bit: 0 */
#define PMMCMD1_L           (0x0020u)  /**< PMM Voltage Regulator Current Mode Bit: 1 */

#endif /* #ifdef  __MSP430_HAS_PMM__ */


#define TESTPRINT_BUF_LEN  256


   //--- shell.c

struct _SHELLINFO; 

typedef int (*sh_exec_cmd_handler)(struct _SHELLINFO * sh_info);
typedef int (*sh_send_str_handler)(char * buf);
typedef void (*sh_send_char_handler)(unsigned char ch);

/**
*
* shell command defenition ( in FLASH )
*/
typedef struct _SHELLCMD
{
   char * cmd_name;                 //!< command name 
   sh_exec_cmd_handler exec_cmd;    //!< command function
}SHELLCMD;

/**
* shell data structure
*
*/
struct _SHELLINFO
{
   int cmd_run;                 //!< command to run number
   int cl_pos;                  //!< parsing cmd line - position
   int cl_size;                 //!< parsing cmd line - size
   unsigned char * cl_buf;      //!< parsing cmd line - pointer to data buffer
   int stop_exec;               //!< stop executing flag 

   unsigned char ** argv;       //!< command lines arguments

   int num_cmd;                 //!< number command line parameters
   SHELLCMD * sh_cmd_arr;       //!< pointer to shell commands

   sh_send_str_handler  sh_send_str;   //!< handler to send string
   sh_send_char_handler sh_send_char;  //!< handler to send char
};

typedef struct _SHELLINFO SHELLINFO;


#define  SH_EXECUTE     8    /**< EXEC command code */
#define  SH_BREAK       3    /**< BREAK command code  */

#define  CTRLC_SYM      3    /**< CTRL+C (break) symbol code  */
#define  BS_SYM         8    /**< ASCII symbol code  */
#define  LF_SYM        10    /**< ASCII symbol code  */
#define  CR_SYM        13    /**< ASCII symbol code  */
#define  DC_1_SYM      17    /**< ASCII symbol code  */
#define  DC_2_SYM      18    /**< ASCII symbol code  */
#define  ESC_SYM       27    /**< ASCII symbol code  */
#define  SPACE_SYM     32    /**< ASCII symbol code  */
#define  DEL_SYM     0x7F    /**< DEL symbol code  */

#define  SH_MAX_ARG                        4    /**< maximum arguments in the shell command line */
#define  SH_CL_SIZE                       96    /**< maximum symbols in the shell command line */
#define  SHELL_UART_RX_PAYLOAD_BUF_SIZE   64    /**< buffer size for uart payload */


/**
*    shell UART driver
*
*/
typedef struct _UARTDRV
{
   unsigned char * buf; //!< pointer to RX buffer  
   int max_buf_size;    //!< buffer size
   int pos;             //!< position in buffer
}UARTDRV;


/**
*  Global data structure with overall shell data
*
*/
typedef struct _BSP_SHELL_DS
{
   SHELLINFO shell_info;                   //!< shell info
   char * shell_argv[SH_MAX_ARG];          //!< pointers to input arguments 
   unsigned char shell_cl_buf[SH_CL_SIZE]; //!< command line input buffer
   UARTDRV sh_uart_drv;                    //!< shell UART driver
   unsigned char sh_uart_drv_payload_buf[SHELL_UART_RX_PAYLOAD_BUF_SIZE]; //!< shell UART driver payload buffer
}BSP_SHELL_DS;


void sh_init(BSP_SHELL_DS * p_sh);
void sh_input(SHELLINFO * sh_info, unsigned char ch);
int sh_start_cmd_exec(SHELLINFO * sh_info);
int sh_do_cmd_exec(SHELLINFO * sh_info);
int sh_prompt(SHELLINFO * sh_info, int send_CR_LF);
int sh_stop_cmd_exec(SHELLINFO * sh_info);
int sh_echo(SHELLINFO * sh_info, unsigned char ch);
int sh_putchar(SHELLINFO * sh_info, unsigned char ch);

//========== System Message ==================================================

#pragma pack(push, 2)

#define  SYS_MSG_PAYLOAD_SIZE    4
typedef struct _SYS_MSG
{
   unsigned short op_code;
   unsigned short sender;
   unsigned short payload_op_code; // 
   union
   {
      unsigned char payload[SYS_MSG_PAYLOAD_SIZE];
      unsigned char * mem_buf;
   };
   unsigned short  mem_buf_len; // if 'mem_buf' is used - > 0, othewise - 0

}SYS_MSG;

#pragma pack(pop)

//----- Sys messages

#define  SYS_MSG_LTT_1_EVENT            10

#define  SYS_MSG_BTN1_PRESSED           20
#define  SYS_MSG_BTN1_RELEASED          21

#define  SYS_MSG_JOB2_STOP              30

#define  SYS_MSG_WAKEUP_BTN0            40

#define  SYS_MSG_INT_BTN0               90  
#define  SYS_MSG_RTC_ALARM              91
#define  SYS_MSG_RTC_1SEC               92



//============================================================================

int tn_dump_int_ex(unsigned char * buf, // output buffer
                   int buf_len,         // total output buffer length
                   const char * fmt,    // format string
                   int val,             // value to dump
                   int width,           // is used only if 'fmp' contains '.*' in the width field
                   int precision);      // is used only if 'fmp' contains '.*' in the precision field

static TN_INLINE int tn_dump_int(unsigned char * buf, // output buffer
                          int buf_len,         // total output buffer length
                          const char * fmt,    // format string
                          int val)             // value to dump
{
   return tn_dump_int_ex(buf, buf_len, fmt, val, 0, 0);
}
//----------------------------------------------------------------------------
//  Converting unsigned short/integer to string according to the format options in
//  'fmt' string (snprintf like, but for the single unsigned short/integer argument 
//----------------------------------------------------------------------------
int tn_dump_uint_ex(const char * fmt, // format string
                    unsigned char * buf, // output buffer
                    int buf_len,      // total output buffer length
                    unsigned int val,          // value to dump
                    int width,        // is used only if 'fmp' contains '.*' in the width field
                    int precision);   // is used only if 'fmp' contains '.*' in the precision field

static TN_INLINE int tn_dump_uint(const char * fmt, // format string
                               unsigned char * buf, // output buffer
                               int buf_len,      // total output buffer length
                               unsigned int val)          // value to dump
{
   return tn_dump_uint_ex(fmt, buf, buf_len, val, 0, 0);
}
    


//----------------------------------------------------------------------------
//  Converting long to string according to the format options in
//  'fmt' string (snprintf like, but for the single long argument 
//----------------------------------------------------------------------------
int tn_dump_long(const char * fmt, // format string
                 unsigned char * buf, // output buffer
                 int buf_len,      // total output buffer length
                 long val,         // value to dump
                 int width,        // valid only if 'fmp' contains '.*' in the width field
                 int precision);   // valid only if 'fmp' contains '.*' in the precision field

//----------------------------------------------------------------------------
//  Converting unsigned long to string according to the format options in
//  'fmt' string (snprintf like, but for the single unsigned long argument 
//----------------------------------------------------------------------------
int tn_dump_ulong(const char * fmt, // format string
                  unsigned char * buf, // output buffer
                  int buf_len,      // total output buffer length
                  unsigned long val,         // value to dump
                  int width,        // valid only if 'fmp' contains '.*' in the width field
                  int precision);   // valid only if 'fmp' contains '.*' in the precision field

//----------------------------------------------------------------------------
//  Converting (unsigned) char to string according to the format options in
//  'fmt' string (snprintf like, but for the single (unsigned) char argument 
//----------------------------------------------------------------------------
int tn_dump_uchar(const char * fmt, // format string
                  unsigned char * buf,  // output buffer
                  int buf_len,       // total output buffer length
                  unsigned char val); // value to dump


/**
 *
 *  Button processing
 */   

#define  BTN_PORT_ADDR                0x0201U    /**<  P2IN */
#define  MAX_BTN_BOUNCE_SUPPR_DLY           4    /**< ~ 15 ms */
#define  BTN_IS_PRESSED_AT_0                1    /**< if logic level is '0' - button is in pressed state */
#define  BTN_IS_PRESSED_AT_1                2    /**< if logic level is '1' - button is in released state */

#define  BTN_ID_1  0
#define  BTN_ID_2  1

#define  BTN_STATE_BTN_ON           1               //!< Button is ON
#define  BTN_STATE_BTN_PRESSED      2               //!< Button is Pressed
#define  BTN_STATE_BTN_OFF          3               //!< Button is OFF
#define  BTN_STATE_BTN_RELEASED     4               //!< Button is Released
#define  BTN_STATE_BTN_UNKNOWN      5               //!< Button state is unknown(after reset)

/**
 *
 *  Callback function prototype to send event
 */   
typedef int (*send_btn_state_func)(int btn, int state);

/**
 *
 *  The button information structure 
 */   
typedef struct _BTNDATA
{                      
   int bounce_cnt;       //!< button debouncing counter
   int max_bounce_cnt;   //!< max button debouncing counter value
   
   unsigned char * port; //!< button CPU I/O port address
   unsigned int pin;     //!< button CPU I/O port bit
   int num;              //!< button number in button enumeration 
   int on_level;         //!< physical level for "Button pressed" state (1 or 1)
      
   int status;           //!< current button state (unknown, pressed, released)   
   send_btn_state_func send_btn_state_handler; //!< callback function to send event to the logical panel driver 
   
}BTNDATA;


void button_processing(void);
void buttons_init(void);

// LEDs processing

#define  LED_PORT_ADDR               0x0223U 

typedef struct _LEDSDATA
{         
   int led_id;
   int led_state;
   int on_level;
   int blink_start_state;

   unsigned int blink_period;
   unsigned int blink_times;
   unsigned int blink_period_cnt;
   unsigned int blink_times_cnt;

   unsigned short port_addr;
   unsigned int pin;
}LEDSDATA;

#define LED_ON_AT_BLINK_START   0
#define LED_OFF_AT_BLINK_START  1

#define LED_STATE_OFF           0
#define LED_STATE_ON            1
#define LED_STATE_ON_BLINK      2
#define LED_STATE_OFF_BLINK     3 

#define NO_BLINK         0U
#define BLINK_INFINITE   0xFFFFU


void leds_init(void);
LEDSDATA * get_leds_arr(void);
void sys_led_processing(void);
void set_led_on(int led_id);
void set_led_off(int led_id);
void set_led_blink(int led_id,
                   int blink_start_state,
                   unsigned int blink_period, // if NO_BLINK - forever, 'blink_times' ignored
                   unsigned int blink_times);

//-------- Deep Sleep

typedef struct _DEEPSLEEPDATA
{
   //Add here your subsystem counter to prevent deep sleep
   //int rtc_op_cnt;
   int led_op_cnt;
   //int job2_cnt;
   int UART_cnt;
   //int rtc_1sec_cnt;

   int cpu_freq_l;               //!< SYS_CPU_FREQ_LO or SYS_CPU_FREQ_HIGH
   int curr_osc;                 //!< OSC_XT1 or OSC_XT2 - set after the change freq/osc function is finished
   unsigned int cpu_freq_val;    //!< in kHz (now - 5000U or 20000U)

}DEEPSLEEPDATA;

DEEPSLEEPDATA * get_deep_sleep_data(void);

/**
 *
 *  Oscillators mode and frequency
 */
#define  OSC_XT1                1            /**< Use XT1 as clocks source */
#define  OSC_XT2                2            /**< Use XT2 as clocks source */

#define  OSC_XT1_1              3            /**< Use XT1 as clocks source */
#define  OSC_XT2_1              4            /**< Use XT1 as clocks source */

#define  OSC_X1_32768HZ         32768UL      /**< XT1 crystal clock */
#define  OSC_X2_4MHZ            4000000UL    /**< XT2 crystal clock (option 1) */
#define  OSC_X2_8MHZ            8000000UL    /**< XT2 crystal clock (option 2) */
#define  OSC_X2_12MHZ           12000000UL   /**< XT2 crystal clock (option 3) */
#define  OSC_X2_20MHZ           20000000UL   /**< XT2 crystal clock (option 3) */

#define  SYS_FREQ_1MHZ          1000U        /**< in kHz  */
#define  SYS_FREQ_2MHZ          2000U        /**< in kHz  */
#define  SYS_FREQ_4MHZ          4000U        /**< in kHz  */
#define  SYS_FREQ_5MHZ          5000U        /**< in kHz  */
#define  SYS_FREQ_6MHZ          6000U        /**< in kHz  */ 
#define  SYS_FREQ_8MHZ          8000U        /**< in kHz  */ 
#define  SYS_FREQ_20MHZ         20000U       /**< in kHz  */

/**
 *
 *  Current CPU frequency
 */
#define  SYS_CPU_FREQ_LO        1    /**< CPU now in low freq mode (MCLK = 5MHz) */
#define  SYS_CPU_FREQ_HIGH      2    /**< CPU now in high freq mode(MCLK = 20MHz) */



#endif
