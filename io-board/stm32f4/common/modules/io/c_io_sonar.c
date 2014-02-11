/**
  ******************************************************************************
  * @file    modules/io/c_io_sonar.c
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    11-Fevereiro-2013
  * @brief   Implementação da leitura do sonar XL-MaxSonar-EZ MB1200.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_sonar.h"

#include "c_common_gpio.h"
#include "c_common_uart.h"

#include <math.h>

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_Sonar
  *	\brief Componente para a leitura do sonar XL-MaxSonar-EZ MB1200.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define SONAR_BAUDRATE  9600
#define SONAR_USART     USART6
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/**
  * @brief  Initializes the sonar module according to the specified
  *         parameters in the define area .
  * @retval None
  */
void c_io_sonar_init()
{
  c_common_usart6_init(9600); //change this command if SONAR_USART change
}

/**
  * @brief  return the distance between the sonar and floor
  * @retval the distance in centimeters
  */

int  c_io_sonar_read()
{
  char dist[3];
  while(c_common_usart_read(USART6)!='R'){}
  dist[0]=c_common_usart_read(USART6);
  dist[1]=c_common_usart_read(USART6);
  dist[2]=c_common_usart_read(USART6);
  return atoi(dist);
}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

