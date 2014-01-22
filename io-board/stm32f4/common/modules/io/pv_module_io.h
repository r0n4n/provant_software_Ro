/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_MODULE_IO_H
#define PV_MODULE_IO_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "c_io_rx24f.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void module_io_init();
void module_io_run();

#ifdef __cplusplus
}
#endif

#endif //PV_MODULE_IO_H
