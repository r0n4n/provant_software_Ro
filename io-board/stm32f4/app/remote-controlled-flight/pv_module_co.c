/**
  ******************************************************************************
  * @file    modules/rc/pv_module_rc.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   ...
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "pv_module_co.h"

/** @addtogroup ProVANT_app
  * @{
  */

/** @addtogroup app_co
  * \brief Módulo com as principais funcionalidades para calculo de controle e escrita de atuadores.
  *
  * Definição do módulo.
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MODULE_PERIOD	 12//ms
#define ESC_ON           1
#define SERVO_ON         1

/* Private macro -------------------------------------------------------------*/
#define USART_BAUDRATE     921600 //<-Beaglebone
/* Private variables ---------------------------------------------------------*/
portTickType pv_module_co_lastWakeTime;
pv_msg_input iInputData;
pv_msg_controlOutput iControlBeagleData;
pv_msg_controlOutput oControlOutputData; 
pv_type_actuation  pv_module_co_actuation;
int pv_module_co_flag;
GPIOPin pv_module_co_LED5;

/* Inboxes buffers */
pv_type_actuation    iActuation;
/* Outboxes buffers*/

/* Private function prototypes -----------------------------------------------*/
unsigned char setPointESC_Forca(float forca);
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializacao do módulo de controle + output.
  *
  * Instancia as Queues de comunicação inter-thread, inicializa a pinagem necessária para
  * os perifericos e aloca o que for necessário para as equações de controle.
  * @param  None
  * @retval None
  */
void module_co_init() 
{
   /* Inicializar os escs*/
  c_common_i2c_init(I2C3);
  c_io_blctrl_init_i2c(I2C3);

  /* Inicializar os servos */
  //feito no module in

  /*USART initialization for beagle comunication*/
  c_common_usart2_init(USART_BAUDRATE);

  /*Inicializar o tipo de controlador*/
  c_rc_BS_control_init();

  /* Pin for debug */
  pv_module_co_LED5 = c_common_gpio_init(GPIOD, GPIO_Pin_14, GPIO_Mode_OUT); //LD5
  /*Data consumed by the thread*/
  pv_interface_co.iInputData          = xQueueCreate(1, sizeof(pv_msg_input));
  pv_interface_co.iControlBeagleData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));
  /*Data produced by the thread*/
  pv_interface_co.oControlOutputData  = xQueueCreate(1, sizeof(pv_msg_controlOutput));

}

/** \brief Função principal do módulo de RC.
  * @param  None
  * @retval None
  *
  * Interpreta o recebimento de PPM, calcula sinais de controle e os envia
  * via interface.
  * Devido as diferenças do modelo matematica com a construção mecanica o sinal do angulo do servo direito deve 
  * ser adaptado.
  *
  */
void module_co_run() 
{
  unsigned int heartBeat=0;
  bool a=0;
  unsigned char sp_right=0;
  pv_module_co_flag=0;
  float rpy[3];
  float drpy[3];
  float position[3];
  float velocity[3];
  float alpha[2];
  float dalpha[2];
  float data1[2],data2[2],data3[2];
  int aux[2];
  float aux2[3];
  float servoTorque[2];
  float escForce[2];
  int channel[7];
  /* Inicializa os dados da attitude*/
  oControlOutputData.actuation.servoRight = 0;
  oControlOutputData.actuation.servoLeft  = 0;
  oControlOutputData.actuation.escRightSpeed = 0;
  oControlOutputData.actuation.escLeftSpeed  = 0;

  while(1) 
  {

	/* Leitura do numero de ciclos atuais */
	  pv_module_co_lastWakeTime = xTaskGetTickCount();

	/* Variavel para debug */
	heartBeat+=1;
	oControlOutputData.heartBeat = heartBeat;

	/* toggle pin for debug */
	//c_common_gpio_toggle(LED_builtin_io);

    /* Passa os valores davariavel compartilha para a variavel iInputData */
	if (uxQueueMessagesWaiting(pv_interface_co.iInputData)!=0){
		xQueueReceive(pv_interface_co.iInputData, &iInputData, 0);
	}
/*
	if (uxQueueMessagesWaiting(pv_interface_co.iControlBeagleData)!=0){
		xQueueReceive(pv_interface_co.iControlBeagleData, &iControlBeagleData, 0);
	}
*/
    /*Calculo do controle*/
	/*No antigo codigo de rodrigo a validaçao do canal B esta mal feita e sempre esta ligado o enableintegration
	 * Neste codigo ja esta corregido mas deijo esse bit sempre ligado no contorle
	 * */
	#ifdef LQR_ATTITUDE_HEIGHT_CONTROL
		iActuation = c_rc_LQR_AH_controller(iInputData.attitude,iInputData.attitude_reference,iInputData.position,iInputData.position_refrence,(float)(iInputData.receiverOutput.joystick[0])/200,iInputData.flightmode);
		// Ajusta o eixo de referencia do servo (montado ao contrario)
		iActuation.servoLeft = -iActuation.servoLeft;
	#elif defined BACKSTEPPING_ATTITUDE_HEIGHT_CONTROL
		iActuation = c_rc_BS_AH_controller(iInputData.attitude,iInputData.attitude_reference,iInputData.position,iInputData.position_refrence,(float)(iInputData.receiverOutput.joystick[0])/200,iInputData.flightmode,iInputData.enableintegration);
		// Ajusta o eixo de referencia do servo (montado ao contrario)
		iActuation.servoLeft = -iActuation.servoLeft;
	#elif defined TORQUE_CONTROL
		aux[0]=iInputData.securityStop;
		aux[1]=0;

		aux2[0]=0;
		aux2[1]=0;
		aux2[2]=0;

		rpy[0]=iInputData.attitude.roll;
		rpy[1]=iInputData.attitude.pitch;
		rpy[2]=iInputData.attitude.yaw;

		drpy[0]=iInputData.attitude.dotRoll;
		drpy[1]=iInputData.attitude.dotPitch;
		drpy[2]=iInputData.attitude.dotYaw;

		position[0]=0;//iInputData.position.x;
		position[1]=0;//iInputData.position.y;
		position[2]=0;//iInputData.position.z;

		velocity[0]=iInputData.position.dotX;
		velocity[1]=iInputData.position.dotY;
		velocity[2]=iInputData.position.dotZ;

		alpha[0]=iInputData.servosOutput.servo.alphal;
		alpha[1]=iInputData.servosOutput.servo.alphar;

		dalpha[0]=iInputData.servosOutput.servo.dotAlphal;
		dalpha[1]=iInputData.servosOutput.servo.dotAlphar;

		data1[0]= oControlOutputData.actuation.servoLeft;
		data1[1]= oControlOutputData.actuation.servoRight;
		data3[0]= oControlOutputData.actuation.escLeftNewtons;
		data3[1]= oControlOutputData.actuation.escLeftSpeed;
		data2[0]= oControlOutputData.actuation.escRightNewtons;
		data2[1]= oControlOutputData.actuation.escRightSpeed;

		/*Send Data to the BeagleBone Black */
		c_common_datapr_multwii2_sendControldatain(rpy,drpy,position,velocity);
		c_common_datapr_multwii2_sendEscdata(aux,alpha,dalpha);
		c_common_datapr_multwii2_sendControldataout(data1,data3,data2);
		c_common_datapr_multwii_debug((float)oControlOutputData.cicleTime,(float)iInputData.cicleTime,(float)oControlOutputData.heartBeat,(float)iInputData.heartBeat);
		c_common_datapr_multwii_sendstack(USART2);

		/*Receives control input data from the beaglebone*/
		pv_module_co_flag=c_common_datapr_multiwii_receivestack(USART2);
		pv_module_co_actuation=c_common_datapr_multwii_getactuation();

		if(pv_module_co_actuation.escLeftNewtons!=0 || pv_module_co_actuation.escRightNewtons!=0 || pv_module_co_actuation.servoLeft!=0 || pv_module_co_actuation.servoRight!=0){
			oControlOutputData.actuation=pv_module_co_actuation;
		}

	#endif

	#ifdef ENABLE_SERVO
	/* Escrita dos servos */

	if (iInputData.securityStop){
		c_io_servos_writePosition(0,0);
	}
	else{
		// inicializacao
		if (iInputData.init){
			c_io_servos_writePosition(0,0);
		}
		else{
			//c_io_servos_writePosition(iActuation.servoRight,iActuation.servoLeft);
			c_io_servos_writeTorque(oControlOutputData.actuation.servoRight,oControlOutputData.actuation.servoLeft);
		}
	}
	#endif

	#ifdef ENABLE_ESC
	/* Escrita dos esc */
	unsigned char sp_right;
	unsigned char sp_left;

	sp_right = setPointESC_Forca(oControlOutputData.actuation.escRightNewtons);
	sp_left = setPointESC_Forca(oControlOutputData.actuation.escLeftNewtons );

	if (iInputData.securityStop){
		c_io_blctrl_setSpeed(1, 0 );//sp_right
		c_common_utils_delayus(10);
		c_io_blctrl_setSpeed(0, 0 );//sp_left
	}
	else{
		if (iInputData.receiverOutput.joystick[0]>50){
			//inicializacao
			if (iInputData.init){
				c_io_blctrl_setSpeed(1, ESC_MINIMUM_VELOCITY);
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(0, ESC_MINIMUM_VELOCITY);
			}
			else{
				c_io_blctrl_setSpeed(1, sp_right );//sp_right
				c_common_utils_delayus(10);
				c_io_blctrl_setSpeed(0, sp_left );//sp_left
			}
		}
	}
	#endif

	/*Time Code Execution*/
	unsigned int timeNow=xTaskGetTickCount();
    oControlOutputData.cicleTime = timeNow - pv_module_co_lastWakeTime;

    /* toggle pin for debug */
    c_common_gpio_toggle(pv_module_co_LED5);
/*
    if(pv_interface_co.oControlOutputData != 0)
      xQueueOverwrite(pv_interface_co.oControlOutputData, &oControlOutputData);
*/
    /* A thread dorme ate o tempo final ser atingido */
    vTaskDelayUntil( &pv_module_co_lastWakeTime, MODULE_PERIOD / portTICK_RATE_MS);
	}
}

/**\ brief Calcula o set point do ESC a partir da forca passada por argumento
 * Curva retirada dos ensaios com os motores brushless no INEP
 */
unsigned char setPointESC_Forca(float forca){
	//	Coefficients:
#ifdef AXI2814
	float p1 = 0.00088809, p2 = -0.039541, p3 = 0.67084, p4 = -5.2113, p5 = 16.33, p6 = 10.854, p7 = 3.0802, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,6) + p2*pow(forca,5) + p3*pow(forca,4) + p4*pow(forca,3)
								+ p5*pow(forca,2) + p6*forca + p7);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
#endif

#ifdef AXI2826
	float p1 = 0.000546, p2 = -0.026944, p3 = 0.508397, p4 = -4.822076, p5 = 35.314598, p6 = 3.636088, set_point=0;

	if (forca <= 0)
		return (unsigned char) ESC_MINIMUM_VELOCITY;
	else{
		set_point = (p1*pow(forca,5) + p2*pow(forca,4) + p3*pow(forca,3) + p4*pow(forca,2)
								+ p5*forca + p6);
	    if (set_point >= 255)
	    	return (unsigned char)255;
	    else
	    	return (unsigned char)set_point;}
#endif
}
/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */
