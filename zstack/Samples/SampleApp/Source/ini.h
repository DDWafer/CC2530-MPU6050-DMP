#ifndef INI_H
#define INI_H
typedef unsigned  char uint8;
typedef unsigned short int uint16;
typedef unsigned  char bool;
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// Maximun number of Vdd samples checked before go on
#define MAX_VDD_SAMPLES  3
#define ZMAIN_VDD_LIMIT  HAL_ADC_VDD_LIMIT_4

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 open_flag=0;  //开机信号 
uint8 close_flag=0;  //关机信号
uint8 timesleep=0;   //定时休眠 
uint8 untimesleep=0;//无休止休眠
uint8 timesleep_count=0; //用于产生1小时定时休眠
uint8 timesleep_test=0;

uint16 count=0; //触摸唤醒后开始计时
int m_state = 0;//0:待机 1：开机 2:关机
extern uint8 count_open;
extern uint8 count_close;
extern uint8 count_open;
extern uint8 count_close;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

extern bool HalAdcCheckVdd (uint8 limit);

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void InitT3(void);
static void zmain_dev_info( void );
static void zmain_ext_addr( void );
static void zmain_vdd_check( void );
 void ini( void );

#ifdef LCD_SUPPORTED
static void zmain_lcd_init( void );
#endif

/*********************************************************************
 * @fn      main
 * @brief   First function called after startup.
 * @return  don't care
 */
//extern uint16 count;
//extern uint8 openflag;



#endif 
