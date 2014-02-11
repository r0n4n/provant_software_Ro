/**
  ******************************************************************************
  * @file    modules/io/c_io_imu.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Funções para IMUs (incialmente baseadas no CIs ITG3205 e ADXL345).
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_io_imu.h"

#include "c_common_gpio.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

#include <math.h>

/** @addtogroup Module_IO
  * @{
  */

/** @addtogroup Module_IO_Component_IMU
  *	\brief Componente para IMU.
  *
  *	Este componente é projetado para implementar funções da IMU da aeronave - leitura
  *	e pré-processamento. A IMU suportada é a baseada nos CIs ITG3205 e ADXL345, mas
  *	outros modelos podem ser incorporados via #define.
  *
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Exported functions definitions --------------------------------------------*/


/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

