/**
  ******************************************************************************
  * @file    modules/io/c_io_rx24f.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do servo RX-24F.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_rx24f.h"

#include "c_common_gpio.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

#include <math.h>

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_RX24F
  *	\brief Componente para o Servo Dynamixel RX-24F.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

// EEPROM AREA  ///////////////////////////////////////////////////////////
#define AX_MODEL_NUMBER_L           0
#define AX_MODEL_NUMBER_H           1
#define AX_VERSION                  2
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_RETURN_DELAY_TIME        5
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CW_ANGLE_LIMIT_H         7
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_CCW_ANGLE_LIMIT_H        9
#define AX_SYSTEM_DATA2             10
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_UP_LIMIT_VOLTAGE         13
#define AX_MAX_TORQUE_L             14
#define AX_MAX_TORQUE_H             15
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18
#define AX_OPERATING_MODE           19
#define AX_DOWN_CALIBRATION_L       20
#define AX_DOWN_CALIBRATION_H       21
#define AX_UP_CALIBRATION_L         22
#define AX_UP_CALIBRATION_H         23

// RAM AREA  //////////////////////////////////////////////////////////////
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_POSITION_H          31
#define AX_GOAL_SPEED_L             32
#define AX_GOAL_SPEED_H             33
#define AX_TORQUE_LIMIT_L           34
#define AX_TORQUE_LIMIT_H           35
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_POSITION_H       37
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_SPEED_H          39
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_LOAD_H           41
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_PAUSE_TIME               45
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48
#define AX_PUNCH_H                  49

// Status Return Levels ///////////////////////////////////////////////////
#define AX_RETURN_NONE              0
#define AX_RETURN_READ              1
#define AX_RETURN_ALL               2

// Instruction Set ////////////////////////////////////////////////////////
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_REG_WRITE                4
#define AX_ACTION                   5
#define AX_RESET                    6
#define AX_SYNC_WRITE               131

// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define AX_BYTE_READ                1
#define AX_BYTE_READ_POS            2
#define AX_ID_LENGTH                4
#define AX_BD_LENGTH                4
#define AX_TEM_LENGTH               4
#define AX_VOLT_LENGTH              4
#define AX_LED_LENGTH               4
#define AX_TORQUE_LENGTH            4
#define AX_POS_LENGTH               4
#define AX_GOAL_LENGTH              5
#define AX_GOAL_SP_LENGTH           7
#define BROADCAST_ID                254
#define AX_START                    255
#define BUFFER_SIZE		  			 64
#define TIME_OUT                    10
#define TX_DELAY_TIME		    	 400

#define PIN_CONTROL_PORT		 	 GPIOC
#define PIN_CONTROL				 	 GPIO_Pin_0
#define RXUSART						 USART6

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIOPin controlPin;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int prv_read_error() {

	return 0;
}
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa USART6 conectada ao barramento dos servos e o pino de controle.
  * Desliga interrupt de recebimento (feito via pooling neste caso).
  *
  * \todo Generalizar a inicialização para qualquer USART.
  *
  * @param  baudrate Baudrate configurada nos servos.
  * @retval None
  */
void c_io_rx24f_init(int baudrate) {
	c_common_usart6_init(baudrate);
	c_common_usart_it_set(RXUSART, USART_IT_RXNE, ENABLE);
	controlPin = c_common_gpio_init(PIN_CONTROL_PORT, PIN_CONTROL, GPIO_Mode_OUT);
}

/** \brief Move o servo para posição desejada, em graus.
  * Retorna \b 1 em caso de sucesso.
  *
  * @param  ID ID do servo.
  * @param  position Posição alvo em graus.
  * @retval Status.
  */
int c_io_rx24f_move(unsigned char ID, int position) {
	int hexPosition = round(c_common_utils_map(position,0,300,0,1023));
    unsigned char Position_H, Position_L;
    Position_H = hexPosition >> 8;
    Position_L = 0x00FF & hexPosition;

    unsigned int TChecksum = (ID +
                             AX_GOAL_LENGTH +
                             AX_WRITE_DATA +
                             AX_GOAL_POSITION_L +
                             Position_L +
                             Position_H);
    if ( TChecksum >= 255){
    	TChecksum = TChecksum & 0xFF;
    }

    unsigned char checksum = 0xFF - TChecksum;

    c_common_gpio_set(controlPin);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, ID);
    c_common_usart_putchar(RXUSART, AX_GOAL_LENGTH);
    c_common_usart_putchar(RXUSART, AX_WRITE_DATA);
    c_common_usart_putchar(RXUSART, AX_GOAL_POSITION_L);
    c_common_usart_putchar(RXUSART, Position_L);
    c_common_usart_putchar(RXUSART, Position_H);
    c_common_usart_putchar(RXUSART, checksum);
    while( !(RXUSART->SR & 0x00000040) );
    c_common_gpio_reset(controlPin);
    //receive answer...

	return 1;
}

/** \brief Move o servo para posição desejada, em graus.
  * Retorna \b 1 em caso de sucesso.
  *
  * @param  ID ID do servo.
  * @param  position Posição alvo em graus.
  * @retval Status.
  */
int c_io_rx24f_setLed(unsigned char ID, unsigned char value) {
    unsigned int TChecksum = (ID +
                             AX_LED_LENGTH +
                             AX_WRITE_DATA +
                             AX_LED +
                             value);
    while ( TChecksum >= 255){
		TChecksum -= 255;
    }

    unsigned char checksum = 0xFF - TChecksum;

    c_common_gpio_set(controlPin);
    for(int i=0xFFFF; i>0; i--);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, ID);
    c_common_usart_putchar(RXUSART, AX_LED_LENGTH);
    c_common_usart_putchar(RXUSART, AX_WRITE_DATA);
    c_common_usart_putchar(RXUSART, AX_LED);
    c_common_usart_putchar(RXUSART, value);
    c_common_usart_putchar(RXUSART, checksum);
    while( !(RXUSART->SR & 0x00000040) );
    c_common_gpio_reset(controlPin);



	return 1;
}

/** \brief Lê a posição atual do servo, em graus.
  * Retorna um valor negativo (\em - erro \em) em caso de falha.
  *
  * @param  ID ID do servo.
  * @retval Posicao em graus.
  */
int  c_io_rx24f_readPosition(unsigned char ID) {
    unsigned int TChecksum = (ID +
                              AX_POS_LENGTH  +
                              AX_READ_DATA +
                              AX_PRESENT_POSITION_L +
                              AX_BYTE_READ_POS);
    while ( TChecksum >= 255){
		TChecksum -= 255;
    }

    unsigned char checksum = 0xFF - TChecksum;

    c_common_gpio_set(controlPin);
    for(int i=0xFFFF; i>0; i--);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, AX_START);
    c_common_usart_putchar(RXUSART, ID);
    c_common_usart_putchar(RXUSART, AX_POS_LENGTH);
    c_common_usart_putchar(RXUSART, AX_READ_DATA);
    c_common_usart_putchar(RXUSART, AX_PRESENT_POSITION_L);
    c_common_usart_putchar(RXUSART, AX_BYTE_READ_POS);
    c_common_usart_putchar(RXUSART, checksum);
    while( !(RXUSART->SR & 0x00000040) );
    c_common_gpio_reset(controlPin);
    //receive answer...

	return 1;
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

