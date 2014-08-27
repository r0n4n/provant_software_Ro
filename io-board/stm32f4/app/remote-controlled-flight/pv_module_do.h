/**
  ******************************************************************************
  * @file    app/remote-controlled-flight/pv_module_do.h
  * @author  Patrick Jose Pereira
  * @version V1.0.0
  * @date    27-August-2014
  * @brief   Implementação do módulo de transmissao de dados para fora do ARM.
  ******************************************************************************/

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_DO_H
#define PV_MODULE_DO_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* FreeRTOS kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* proVANT includes */
#include "pv_typedefs.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_do_init();
void module_do_run();

#ifdef __cplusplus
}
#endif

#endif 
