/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Definição do módulo de controle + output.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_CO_H
#define PV_MODULE_CO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "c_common_gpio.h"
#include "c_common_i2c.h"
#include "c_io_blctrl.h"
#include "c_io_servos.h"
#include "c_common_uart.h"
#include "c_common_utils.h"

/*Control includes*/
#include "c_rc_BS_control.h"
#include "c_rc_LQR_control.h"

#include "pv_typedefs.h"
#include "c_datapr_MultWii.h"


/* Exported types ------------------------------------------------------------*/
struct pv_interface_co 
{
  xQueueHandle iInputData;  
  xQueueHandle oControlOutputData;
  xQueueHandle iControlBeagleData;

} pv_interface_co;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define ESC_MINIMUM_VELOCITY	10//esc set point value (0-255)
//#define ENABLE_SERVO
#define AXI2826
//#ENABLE_BEAGLE
//#define AXI2814
//#define ENABLE_ESC
/* Exported functions ------------------------------------------------------- */
void module_co_init();
void module_co_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_CO_H
