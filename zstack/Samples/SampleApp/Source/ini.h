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

uint8 open_flag=0;  //�����ź� 
uint8 close_flag=0;  //�ػ��ź�
uint8 timesleep=0;   //��ʱ���� 
uint8 untimesleep=0;//����ֹ����
uint8 timesleep_count=0; //���ڲ���1Сʱ��ʱ����
uint8 timesleep_test=0;

uint16 count=0; //�������Ѻ�ʼ��ʱ
int m_state = 0;//0:���� 1������ 2:�ػ�
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
