/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_Hinfload_control.h"
#include "c_common_uart.h"
//#include "c_datapr_hilProtocol.h"
#include "c_common_utils.h"

/** @addtogroup Module_RC
  * @{
  */

/** @addtogroup Module_RC_Component_State_Feedback_Control
  * \brief Controle de estabilização para o modo de operação RC por realimentacao de estados.
  *
   * @{
  */

		//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/




pv_type_actuation actuation_signals;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/




/* Exported functions definitions --------------------------------------------*/

void c_rc_HinfLoad_control_init()
{
	// inicializa matrizes estáticas
	
	
}

pv_type_actuation c_rc_HinfLoad_controller(pv_msg_input input)
{

	/*float32_t Hinfload_K_f32_hil[OUTPUT_SIZE][INPUT_STATE_SIZE]=
{{-0.000509474023994 ,  1.381002006541810 ,  2.044930990723325 , -4.098388643419657 ,  0.002544968427177 ,  0.065243786421189, -0.011997162152724 ,  0.012231188237446 , -0.000191109567030 ,  0.977046060078436  , 2.067519443836474 , -1.069820095142832,
0.003981280957280 ,  0.048837474399233 , -0.000157117358773  , 0.000160465562387 , -0.000324184653490 ,  0.932396154093819,
0.987708499654750 ,  0.029590812630684},

{0.000485844262540 , -1.379777079391250 ,  2.046778453514912 ,  4.095526259295323 ,  0.006483505671600 , -0.065184427882726,
0.011988462007403 , -0.012218371367099 , 0.000647666101473 , -0.976219155907380 ,  2.069372953111319  , 1.069647999496566,
0.005073107886213 , -0.048792319825525 ,  0.000156410506717 , -0.000160891309965 ,  0.000193352263628 , -0.931559941080183,
0.988594878555003 , -0.029564069576714},

{0.147511637570957 , -0.044700625197703  , 0.000027359219549  , 0.100038920576059  , 0.186220549413359 ,  0.044874312445416,
0.218427852703492 ,  0.036876304589717 ,  0.070402232740872 , -0.028416542202667 ,  0.000007473706141 ,  0.017792742949581,
0.025919243351085 ,  0.030976534992245 ,  0.007155474366137 ,  0.000723596619702 ,  0.140517760855087 , -0.032497295109449,
0.000009086030883 ,  0.021343249034188},

{0.147480373299421 ,  0.044731365984124 ,  0.000026937597506 , -0.099883806663005 ,  0.186155931716280 , -0.044878136021767,
0.036878957916608 ,  0.218443336829840 ,  0.070383903601564 ,  0.028402976825487  , 0.000007312089313 , -0.017740572220967,
0.025908596148404,  -0.030981560936545  , 0.000723658016145  , 0.007155720063258  , 0.140489293081162 ,  0.032562443693621,
0.000009316184096 , -0.021344301330953}
}; */

float32_t Hinfload_K_f32_hil[OUTPUT_SIZE][STATE_SIZE]={{0.7101244468428165 , 6.141471260155515 , 7.765440795725267 , -18.97385920336097 , 1.826066960066656 , -0.09338611048322061 ,
  17.57791369983566 , -0.7164683413615529 , -0.03350735071807359 , 0.4235128078145839 , 0.2708588223207514 , 4.311055003650324 ,
  4.170146360015496 , -5.072227698361692 , 0.3809539101204998 , -0.06650607548831029 , 2.122392130887405 , -0.02473442956879733 ,
  0.0006404185518007683 , 0.008513558472367464},
{0.4662177448319676 , 3.523707063368016 , 5.320429316782071 , -0.1472247707396855 , -1.571274680558446 , -8.596271330525751 ,
    6.985745062499001 , 21.91480875426527 , -1.94921538690414 , 0.09535891280953998 , -18.42904004957552 , 0.4434387926540275 ,
    0.1072202681687932 , -0.317249054464631 , -0.9548973567439005 , -5.910676814456982 , 4.55703288130231 , 5.454379896200787,
    -0.2518284755625079 , 0.06360770401465654},
{-2.133681658482248 , -0.008445306788653999 , 0.001718825918710477 , -0.005954099368496411 , -1.2250793041699 , -5.503330766582297,
    3.925043415306752 , 0.09204438852464802 , 0.1952075590979948 ,-0.03032542746108816 ,-0.003380038380425396 , 0.06292464197787898,
    0.3808490158409268 , 0.04098134842638689 , -0.0437294922542443 , -0.1815063858105274 , 0.1672299855993115 , -0.004950772796723103,
    0.1160491338776613,-0.01901526180028523}
} ;


	float32_t error_state_vector_f32_hil[STATE_SIZE]={0};
	float32_t control_output_f32_hil[OUTPUT_SIZE]={0};
	float32_t state_vector_f32_hil[STATE_SIZE]={0};
	float32_t reference_f32_hil[STATE_SIZE]={0};
	float32_t angular_velocity[3]={0} ;

	arm_matrix_instance_f32 Hinfload_K_hil;
	arm_matrix_instance_f32 error_state_vector_hil;
	arm_matrix_instance_f32 control_output_hil;
	arm_matrix_instance_f32 state_vector_hil;
	arm_matrix_instance_f32 reference_hil;	



	// mémoria para integradores
	static float32_t xint = 0;
	static float32_t x_ant = 0;
	static float32_t yint= 0;
	static float32_t y_ant = 0;
	static float32_t zint= 0;
	static float32_t z_ant = 0;
	static float32_t yawint = 0;
	static float32_t yaw_ant = 0;

	float32_t x_atual = 0;
	float32_t y_atual = 0;
	float32_t z_atual = 0;
	float32_t yaw_atual = 0;

	// criando vetor de refência
	reference_f32_hil[0]= 2;
	reference_f32_hil[1]= 0;
	reference_f32_hil[2]= 1;
	reference_f32_hil[3]= 0;
	reference_f32_hil[4]= 0;
	reference_f32_hil[5]= 0;
	reference_f32_hil[6]= 0.00002965;
	reference_f32_hil[7]= 0.004885;
	reference_f32_hil[8]= 0.004893;
	reference_f32_hil[9]= 0.00484;
	reference_f32_hil[10]= 0;
	reference_f32_hil[11]= 0;
	reference_f32_hil[12]= 0;
	reference_f32_hil[13]= 0;
	reference_f32_hil[14]= 0;
	reference_f32_hil[15]= 0;
	reference_f32_hil[16]= 0;
	reference_f32_hil[17]= 0;
	reference_f32_hil[18]= 0;
	reference_f32_hil[19]= 0;
	reference_f32_hil[20]= 0;
	reference_f32_hil[21]= 0;
	reference_f32_hil[22]= 0;
	reference_f32_hil[23]= 0;

	//Frame teste = frame_create();
	//for(int i=0; i<24;i++) frame_addFloat(&teste, reference_f32_hil[i]);
	//frame_build(&teste);
	//c_io_protocolP2P_send(&teste);
	
	/* Calculate the angular velocity */
	pqr2EtaDot( angular_velocity , input.attitude.dotRoll , input.attitude.dotPitch,  input.attitude.dotYaw , input.attitude.roll, input.attitude.pitch, input.attitude.yaw) ;


	// Integrador Trapezoidal
	x_atual = input.position.x - input.position_reference.x;
	xint = xint + (0.012/2.0)*(x_atual + x_ant);
	x_ant = x_atual;
	y_atual = input.position.y - input.position_reference.y;
	yint = yint + (0.012/2.0)*(y_atual + y_ant);
	y_ant = y_atual;
	z_atual = input.position.z - input.position_reference.z;
	zint = zint + (0.012/2.0)*(z_atual + z_ant);
	z_ant = z_atual;
	yaw_atual = input.attitude.yaw;
	yawint = yawint + (0.012/2.0)*(yaw_atual + yaw_ant);
	yaw_ant = yaw_atual;
	
	// criando vetor de estado
	
	state_vector_f32_hil[STATE_X]= input.position.x;
	state_vector_f32_hil[STATE_Y]= input.position.y;
	state_vector_f32_hil[STATE_Z]= input.position.z;
	state_vector_f32_hil[STATE_ROLL]= input.attitude.roll;
	state_vector_f32_hil[STATE_PITCH]= input.attitude.pitch;
	state_vector_f32_hil[STATE_YAW]= input.attitude.yaw;
	state_vector_f32_hil[STATE_ALPHA_R+2]= input.servosOutput.servo.alphar;
	state_vector_f32_hil[STATE_ALPHA_L+2]= input.servosOutput.servo.alphal;
	state_vector_f32_hil[STATE_LOAD_X_ANGLE-2]= input.load_attitude.x_angle;
	state_vector_f32_hil[STATE_LOAD_Y_ANGLE-2]= input.load_attitude.y_angle ;
	state_vector_f32_hil[STATE_DX]= input.position.dotX;
	state_vector_f32_hil[STATE_DY]= input.position.dotY;
	state_vector_f32_hil[STATE_DZ]= input.position.dotZ;
	state_vector_f32_hil[STATE_DROLL]= angular_velocity[0];/* +
			      input.attitude.dotYaw*cos(input.attitude.roll)*tan(input.attitude.pitch) + 
			      input.attitude.dotPitch*sin(input.attitude.roll)*tan(input.attitude.pitch); */
	state_vector_f32_hil[STATE_DPITCH]= angular_velocity[1];/**cos(input.attitude.roll) -
			      input.attitude.dotYaw*sin(input.attitude.roll); */
	state_vector_f32_hil[STATE_DYAW]= angular_velocity[2];/* *cos(input.attitude.roll))/cos(input.attitude.pitch) +
			      (input.attitude.dotPitch*sin(input.attitude.roll))/cos(input.attitude.pitch); */
	state_vector_f32_hil[STATE_DALPHA_R+2]= input.servosOutput.servo.dotAlphar;
	state_vector_f32_hil[STATE_DALPHA_L+2]= input.servosOutput.servo.dotAlphal;

	state_vector_f32_hil[STATE_LOAD_X_ANGLE_VELOCITY-2]= input.load_attitude.dotx_angle;
	state_vector_f32_hil[STATE_LOAD_Y_ANGLE_VELOCITY-2]= input.load_attitude.doty_angle ;
	/*c_common_usart_puts(USART6, "\ndAR: ");
	char _buff[10];
	c_common_utils_floatToString(state_vector_f32_hil[15], _buff, 0);
	c_common_usart_puts(USART6, _buff);
	state_vector_f32_hil[16]= input.servosOutput.servo.dotAlphal;
	c_common_usart_puts(USART6, "\ndAL: ");
	char _buff2[10];
	c_common_utils_floatToString(state_vector_f32_hil[16], _buff2, 0);
	c_common_usart_puts(USART6, _buff2);*/

  yaw_ant = yaw_atual;
	state_vector_f32_hil[STATE_XINT]= xint;
	state_vector_f32_hil[STATE_YINT]= yint;
	state_vector_f32_hil[STATE_ZINT]= zint;
	state_vector_f32_hil[STATE_YAWINT]= yawint;
	/*state_vector_f32_hil[17]= 0;
	state_vector_f32_hil[18]= 0;
	state_vector_f32_hil[19]= 0;
	state_vector_f32_hil[20]= 0;*/

	//Frame teste2 = frame_create();
	//for(int i=0; i<24;i++) frame_addFloat(&teste2, state_vector_f32_hil[i]);
	//frame_build(&teste2);
	//c_io_protocolP2P_send(&teste2);

	arm_mat_init_f32(&Hinfload_K_hil, OUTPUT_SIZE,STATE_SIZE,(float32_t *)Hinfload_K_f32_hil);
	arm_mat_init_f32(&error_state_vector_hil, STATE_SIZE,1,(float32_t *)error_state_vector_f32_hil);
	arm_mat_init_f32(&control_output_hil,OUTPUT_SIZE,1,(float_t *)control_output_f32_hil);
	arm_mat_init_f32(&state_vector_hil,STATE_SIZE,1,(float_t *)state_vector_f32_hil);
	arm_mat_init_f32(&reference_hil,STATE_SIZE,1,(float_t *)reference_f32_hil);

	// Err = X - Xref
	arm_mat_sub_f32(&state_vector_hil, &reference_hil, &error_state_vector_hil);

	// Out = - K * Err
	arm_mat_mult_f32(&Hinfload_K_hil, &error_state_vector_hil, &control_output_hil);
	
	float32_t input1 = control_output_hil.pData[0];
	float32_t input2 = control_output_hil.pData[1];
	float32_t input3 = control_output_hil.pData[2];
	float32_t input4 = control_output_hil.pData[3];
	
	control_output_hil.pData[0] = (-input1) + 12.6005;
	control_output_hil.pData[1] = (-input2) + 12.609;
	control_output_hil.pData[2] = (-input3) + 0;
	control_output_hil.pData[3] = (-input4) + 0;
	


	actuation_signals.escRightNewtons=(float32_t)control_output_hil.pData[0];
	actuation_signals.escLeftNewtons=(float32_t)control_output_hil.pData[1];
	actuation_signals.servoRight=(float32_t)control_output_hil.pData[2];
	actuation_signals.servoLeft=(float32_t)control_output_hil.pData[3];

	//SendData(state_vector_f32_hil,24) ;

	return actuation_signals;


}

void pqr2EtaDot(float32_t* angular_velocity , double in_a, double in_b, double in_c, double phi, double theta, double psii)
{
  angular_velocity[0] = in_a + in_c*cos(phi)*tan(theta) + in_b*sin(phi)*tan(theta) ;
  angular_velocity[1] = in_b*cos(phi) - in_c*sin(phi) ;
  angular_velocity[2] = (in_c*cos(phi))/cos(theta) + (in_b*sin(phi))/cos(theta) ;

}

/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

