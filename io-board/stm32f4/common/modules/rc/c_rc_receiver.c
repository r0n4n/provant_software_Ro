/**
  ******************************************************************************
  * @file    modules/rc/c_rc_receiver.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implementação do receiver do controle de rádio manual.
  * 		 Implementa as funções de recebimento, detecção e interpretação do
  * 		 receiver configurado em modo PPM.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_receiver.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_Receiver
  * \brief Funções para inicialização e recebimento de sinais PPM do receiver do controle remoto.
  *
  * @{
  */


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define 	EXTI_PORT		EXTI_PortSourceGPIOG
#define 	EXTI_SOURCE		EXTI_PinSource13		// PD5 na placa STM32F4-H407
#define 	EXTI_LINE		EXTI_Line13				// Tem que ser a mesma do pino
#define 	PULSE_INTERVAL 	400						// us
#define 	SYNC_WIDTH		2500					// us
#define	 	NUM_OF_CHANNELS 6

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
long int 	channels[12];
int 		channel_index = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a leitura de PPM num pino predefinido via DEFINE.
  * Inicializa o TIM2 e a interrupção EXTI. A partir deste momento, a contagem
  * de pulsos já está ocorrendo no background.
  *
  * @param  None
  * @retval None
  */
void c_rc_receiver_init() {
	 /* zerando os contadores */
	 for(int i=0; i<12; i++)
		 channels[i] = 0;

	 /* Enable SYSCFG clock */
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	 /* Connect EXTI Line to appropriate GPIO Pin */
	 SYSCFG_EXTILineConfig(EXTI_PORT, EXTI_SOURCE);

	 NVIC_InitTypeDef   NVIC_InitStructure;
	 EXTI_InitTypeDef   EXTI_InitStructure;

	 /* Configure EXTI Line */
	 EXTI_InitStructure.EXTI_Line = EXTI_LINE;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Apenas borda de subida
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure);

	 /* Enable and set EXTI Line Interrupt */
	 NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);

	 /* TIM2 clock enable */
	 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	 /* Time base configuration */
	 TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 2000000) - 1; // a cada us
	 TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;// 1 us
	 TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	 TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	 /* TIM IT enable */
	 TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	 /* TIM2 enable counter */
	 TIM_Cmd(TIM2, ENABLE);
}

/** \brief Retorna a largura do pulso em \em us do canal selecionado.
  * Retorna -1 caso o canal desejado não seja válido (por ex., não exista)
  *
  * @param  int Canal a ser lido (começando em 0)
  * @retval int Duração em \em us do pulso no canal selecionado.
  */
int  c_rc_receiver_get_channel(int channel_n) {
	if(channel_n < NUM_OF_CHANNELS) {
		return channels[channel_n];
	}
	else
		return -1;
}

/* IRQ handlers ------------------------------------------------------------- */
void  EXTI15_10_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_LINE); // clear interrupt
	int pulse_width = TIM_GetCounter(TIM2) - PULSE_INTERVAL;

	if(pulse_width > SYNC_WIDTH) //sync pulse
		channel_index = 0;
	else {
		if(channel_index < NUM_OF_CHANNELS) {
			channels[channel_index] = pulse_width;
			channel_index++;
		}
	}
	TIM_SetCounter(TIM2, 0);
}

/**
  * @}
  */

/**
  * @}
  */

