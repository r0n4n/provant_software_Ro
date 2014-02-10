/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Definição do módulo de controle e comunicação via rádio manual.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_RC_H
#define PV_MODULE_RC_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "c_rc_control.h"
#include "c_rc_receiver.h"

/* Exported types ------------------------------------------------------------*/
struct pv_interface_rc {
	xQueueHandle oThrottles;
	xQueueHandle oAngularRefs;
};


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_RC_init();
void module_RC_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_RC_H
