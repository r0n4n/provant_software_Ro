/**
  ******************************************************************************
  * @file    modules/common/c_common_utils.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Funções gerais para utilização em outros módulos.
  *
  *****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_utils.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_Utils
  *
  * \brief Implementa funções utilitárias para o projeto.
  *
  * Aqui são adicionadas funções genéricas (matemáticas, etc.) passíveis de utilização
  * em todos os módulos.
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

/**brief Função de mapeamento de variável de um intervalo a outro.
 *
 * Mapeia a variável x de um intervalo [ \b in_min , \b in_max ] para [ \b out_min , \b out_max ].
 */
long c_common_utils_map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
  * @}
  */

/**
  * @}
  */

