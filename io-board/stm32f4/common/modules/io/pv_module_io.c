/**
  ******************************************************************************
  * @file    modules/io/pv_module_io.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    02-Dezember-2013
  * @brief   Implementação do módulo de gerenciamento de sensores e atuadores.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_io.h"

/** @addtogroup ProVANT_Modules
  * @{
  */

/** @addtogroup Module_IO
  * \brief Componentes para atuação e sensoriamento do VANT.
  *
  * Reunião de todos os componentes relacionados às operações de I/O do VANT.
  * Leituras de todos os sensores, comandos para atuadores. O processamento destes
  * dados brutos NÃO é feito neste módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	   10//ms

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
portTickType lastWakeTime;
int  accRaw[3], gyroRaw[3], magRaw[3];
char str[256];

/* Inboxes buffers */
pv_msg_io_actuation    iActuation;

/* Outboxes buffers*/
pv_msg_datapr_attitude oAttitude;
pv_msg_datapr_position oPosition;

/* Private function prototypes -----------------------------------------------*/
void start_pwm()
{
	#if 1
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	/////////////////////////////////////////////////////////////////PF6 TIMER10
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6; //PF6 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10); 

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 10 us resolution.
	// Solving for prescaler gives 320 for 32 MHz device

	//timer 168Mhz
	TIM_TimeBaseInitStruct.TIM_Prescaler = (SystemCoreClock/1000000); // 100 KHz 
	TIM_TimeBaseInitStruct.TIM_Period = 19000;   // 0..999, 100 Hz (us)
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStruct);

	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)
	TIM_OC1Init(TIM10, &TIM_OCInitStruct);
	TIM_OC2Init(TIM10, &TIM_OCInitStruct);
	TIM_Cmd(TIM10, ENABLE);
	#endif
	/////////////////////////////////////////////////////////////////PF7 TIMER11
	#if 1
	GPIO_InitTypeDef GPIO_InitStructure2;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct2;
	TIM_OCInitTypeDef TIM_OCInitStruct2;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	
	GPIO_InitStructure2.GPIO_Pin =  GPIO_Pin_9; //PF6 
	GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure2.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure2.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOF, &GPIO_InitStructure2);

	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14); 

	// Let PWM frequency equal 100Hz.
	// Let period equal 1000. Therefore, timer runs from zero to 1000. Gives 10 us resolution.
	// Solving for prescaler gives 320 for 32 MHz device

	//timer 84Mhz
	TIM_TimeBaseInitStruct2.TIM_Prescaler = (SystemCoreClock/2000000); // 100 KHz 
	TIM_TimeBaseInitStruct2.TIM_Period = 19000;   // 0..999, 100 Hz (us)
	TIM_TimeBaseInitStruct2.TIM_ClockDivision = 0;
	TIM_TimeBaseInitStruct2.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct2);

	TIM_OCStructInit(&TIM_OCInitStruct2);
	TIM_OCInitStruct2.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct2.TIM_OCMode = TIM_OCMode_PWM1;
	// Initial duty cycle equals 0%. Value can range from zero to 1000.
	TIM_OCInitStruct2.TIM_Pulse = 0; // 0 .. 1000 (0=Always Off, 1000=Always On)
	TIM_OC1Init(TIM14, &TIM_OCInitStruct2);
	TIM_OC2Init(TIM14, &TIM_OCInitStruct2);
	TIM_Cmd(TIM14, ENABLE);
	#endif
	
}
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao componentes de IO.
  *
  * Incializa o hardware para comunicar com os sensores e atuadores. Rotinas de teste
  * ainda precisam ser executadas.
  * @param  None
  * @retval None
  */
void module_io_init() {
	/* Inicialização do hardware do módulo */
	c_common_i2c_init(I2C1); //imu 
	//c_common_i2c_init(I2C2); //esc

	c_common_usart2_init(460800);

	/* Inicializar do sonar */
	c_io_sonar_init();

	/* Inicializar os servos */
	c_io_rx24f_init(1000000);
	c_common_utils_delayms(2);
	c_io_rx24f_setSpeed(1, 20);
	c_io_rx24f_setSpeed(2, 20);
	c_common_utils_delayms(2);
	/* CCW Compliance Margin e CCW Compliance margin */
	c_io_rx24f_write(1, 0x1A,0x03);
	c_io_rx24f_write(1, 0x1B,0x03);
	c_io_rx24f_write(2, 0x1A,0x03);
	c_io_rx24f_write(2, 0x1B,0x03);
	c_common_utils_delayms(2);
	c_io_rx24f_move(1, 130);
	c_io_rx24f_move(2, 150);
	c_common_utils_delayms(100);

	c_io_imu_init(I2C1);   
	//c_io_blctrl_init(I2C1);

	//inicializacao dos PWM
	start_pwm();

	/* Inicialização das filas do módulo. Apenas inboxes (i*!) são criadas! */
	pv_interface_io.iActuation = xQueueCreate(1, sizeof(pv_msg_io_actuation));

	/* Inicializando outboxes em 0 */
	pv_interface_io.oAttitude = 0;
	pv_interface_io.oPosition = 0;

	/* Verificação de criação correta das filas */
	if(pv_interface_io.iActuation == 0) {
		vTraceConsoleMessage("Could not create queue in pv_interface_io!");
		while(1);
	}
}

/** \brief Função principal do módulo de IO.
  * @param  None
  * @retval None
  *
  * Loop que amostra sensores e escreve nos atuadores como necessário.
  *
  */
void module_io_run() 
{
	float accRaw[3], gyrRaw[3], magRaw[3];
	char  ax[16], ay[16], az[16], r[16], p[16], y[16], dr[16], dp[16], dy[16];
	float rpy[] = {0,0,0,0,0,0};
	int counte=0;

	while(1)
	{
		lastWakeTime = xTaskGetTickCount();

		xQueueReceive(pv_interface_io.iActuation, &iActuation, 0);

    	/*
    	TIM11->CCR1 = 512; 
    	TIM11->CCR2 = 1024 - velo_left*4; 
    	*/
		
		/// IMU DATA
		#if 1
		 	taskENTER_CRITICAL();
			c_io_imu_getComplimentaryRPY(rpy);
			taskEXIT_CRITICAL();
		#endif

		/// SERVOS
		#if 1		
			if(iActuation.servoRight*RAD_TO_DEG<60 || iActuation.servoRight*RAD_TO_DEG>-60)
				c_io_rx24f_move(2, 150+iActuation.servoRight*RAD_TO_DEG);
			if(iActuation.servoLeft*RAD_TO_DEG<60 || iActuation.servoLeft*RAD_TO_DEG>-60)
				c_io_rx24f_move(1, 130+iActuation.servoLeft*RAD_TO_DEG);	
		#endif


		/// ESCS
		#if 1    	
			// força para char
			unsigned char velo_right = (int)(12.256*iActuation.escRightSpeed - 39.441); 
			unsigned char velo_left  = (int)(12.256*iActuation.escLeftSpeed  - 39.441);

			taskENTER_CRITICAL();
			#if 0
				//c_io_blctrl_setSpeed(0, 10 );
				//c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(1, 10 );
			#else
				if(counte>2500)
				{
					TIM10->CCR1 = 1000 + velo_right*3.92;//(unsigned int)(velo_right*3.92); 
			    	TIM10->CCR2 = 1000 - velo_right*3.92;

			    	TIM14->CCR1 = 1000 + velo_left*3.92;
			    	TIM14->CCR2 = 1000 - velo_left*3.92;
			    }
			    else
			    {
				    TIM10->CCR1 = 950 + counte/5.0; 
				    TIM10->CCR2 = 19000-950 - counte/5.0; 	

				    TIM14->CCR1 = 950 + counte/5.0; 
				    TIM14->CCR2 = 19000-950 - counte/5.0; 	
			    }
	    	#endif
			taskEXIT_CRITICAL();
		#endif
		
		/// SONAR
		#if 0
			c_common_utils_floatToString(c_io_sonar_read(), r,  3);
			sprintf(str, "Distance: %s \n\r",r );
	    	c_common_usart_puts(USART2, str);
    	#endif


		/// DEBUG
		#if 1
			//c_common_utils_floatToString(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG, r,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_PITCH ]*RAD_TO_DEG, p,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_YAW   ]*RAD_TO_DEG, y,  4);
			//c_common_utils_floatToString(rpy[PV_IMU_DROLL ]*RAD_TO_DEG, dr, 4);
			//c_common_utils_floatToString(rpy[PV_IMU_DPITCH]*RAD_TO_DEG, dp, 4);
			//c_common_utils_floatToString(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG, dy, 4);
			sprintf(str, "imu -> \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n\r" ,(int)(rpy[PV_IMU_ROLL  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_PITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_YAW  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DROLL  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DPITCH  ]*RAD_TO_DEG), (int)(rpy[PV_IMU_DYAW  ]*RAD_TO_DEG), (int)velo_right , (int)velo_left,(int)(iActuation.servoRight*RAD_TO_DEG), (int)(iActuation.servoLeft*RAD_TO_DEG), counte);
			c_common_usart_puts(USART2, str);
			counte++;
		#endif

		/// DADOS OUT
		oAttitude.roll     = rpy[PV_IMU_ROLL  ];
		oAttitude.pitch    = rpy[PV_IMU_PITCH ];
		oAttitude.yaw      = rpy[PV_IMU_YAW   ];
		oAttitude.dotRoll  = rpy[PV_IMU_DROLL ];
		oAttitude.dotPitch = rpy[PV_IMU_DPITCH];
		oAttitude.dotYaw   = rpy[PV_IMU_DYAW  ];
		if(pv_interface_io.oAttitude != 0)
      		xQueueOverwrite(pv_interface_io.oAttitude, &oAttitude);

		//vTaskDelayUntil( &lastWakeTime, (MODULE_PERIOD / portTICK_RATE_MS));
	}
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

