/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_rc.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_RC
  * \brief Módulo com as principais funcionalidades para operação em modo rádio controlado.
  *
  * Definição do módulo de controle e comunicação via rádio manual.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de RC.
  *
  * Inicializa o receiver e equações de controle.
  * @param  None
  * @retval None
  */
void module_RC_init() {

}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  */
void module_RC_run() {

}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
