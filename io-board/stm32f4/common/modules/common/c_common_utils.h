/**
  ******************************************************************************
  * @file    modules/common/c_common_utils.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Funções gerais para utilização em outros módulos.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_UTILS_H
#define C_COMMON_UTILS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define PV_CONSTANT_G    9.81 //gravity

/* Exported macro ------------------------------------------------------------*/
#define C_COMMON_UTILS_1MS_DELAY for(int i=0; i--; i<168000) { __asm("NOP"); }

/* Exported functions ------------------------------------------------------- */
float c_common_utils_map(float x, float in_min, float in_max, float out_min, float out_max);
float c_common_utils_sat(float x, float min, float max);
void  c_common_utils_delayms(int  ms);
void  c_common_utils_delayus(long us);

/* Header-defined wrapper functions ----------------------------------------- */


#ifdef __cplusplus
}
#endif

#endif //C_COMMON_UTILS_H

