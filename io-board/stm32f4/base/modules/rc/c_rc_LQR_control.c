/**
  ******************************************************************************
  * @file    modules/rc/c_rc_SFC.c
  * @author  Rodrigo Donadel
  * @version V1.0.0
  * @date    08-December-2014
  * @brief   Controle de estabilizacao por realimentacao de estados.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_LQR_control.h"

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

float stepSize[4]={0.1,0.3,0.3,0.1} ;

pv_type_stability_error LQR_AH_integrated_error={0};
pv_type_stability_error LQR_AH_last_error={0};

pv_type_pathtrack_error LQR_PT_integrated_error={0} ;
pv_type_pathtrack_error LQR_PT_last_error={0} ;

// Matrizes de ganho
//float32_t LQR_AH_Ke_f32[4][8]={
//{21.207062455586751, -24.707408251121784,   0.069346468761083,   2.452130918218125,  12.216788626480916,  -4.011929550716840,  0.004475317798479,   0.557055453783041},
//{21.233444539785776,  24.686653404459395,  -0.003353700697260,  -2.445515972875630,  12.233162353241495,   4.011111755550325,  0.005166788138636,  -0.555396333501935},
//{-0.000719994314509,   0.171143370645070,   0.344766866536217,   2.043259478401853,  -0.000386473018984,   0.031030138533026,  0.348169765415673,   0.506637304021608},
//{-0.000523964029942,  -0.167138107269209,   0.375284925986611,  -2.029180987056317,  -0.000281018829228,  -0.030178402624864,  0.345138150637528,  -0.506235008031713}
//};
//float32_t LQR_AH_Ki_f32[4][8]={
//{-18.360708418541652,  74.747291643686452,  -0.082091356993229,  -5.151947191333180},
//{-18.381627935970801, -74.663218767292690,   0.029590624926112,   5.137988576748046},
//{0.000669511614153,   -0.467900642908488,   -4.063031942899512,  -4.032256152392106},
//{0.000487812904700,   0.459532421289280,    -4.048507830710655,   4.047414146624359}
//};
//float32_t LQR_AH_Ke_f32[4][8]={
//    {7.2736,   -7.8155,    0.0283,   -0.3765,    5.7900,   -1.6030,    0.0056,   -0.1355},
//    {7.2715,    7.7557,    0.0077,    0.3739,    5.7898,    1.5958,    0.0041,    0.1348},
//    {-0.0047,   -0.4258,    4.5543,    3.4167,   -0.0033,   -0.0408,    0.9586,    1.0033},
//    {-0.0049,    0.4375,    4.5827,   -3.3797,   -0.0035,    0.0430,    0.9544,   -1.0018}
//};
float32_t LQR_AH_Ki_f32[4][4]={
   {-4.5685,   10.2205,   -0.0282,    0.5649},
   {-4.5660,  -10.1532,   -0.0040,   -0.5610},
   {0.0034,    0.4810,   -9.6865,   -5.2854},
   {0.0035,   -0.4932,   -9.6756,    5.2928}
};

float32_t LQR_PT_Ki_f32[4][4]={
   {-0.000324184653490 ,  0.932396154093819, 0.987708499654750 ,  0.029590812630684},
   {0.000193352263628 , -0.931559941080183, 0.988594878555003 , -0.029564069576714},
   {0.140517760855087 , -0.032497295109449, 0.000009086030883 ,  0.021343249034188},
   {0.140489293081162 ,  0.032562443693621, 0.000009316184096 , -0.021344301330953}
};


// bom
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02577777219334,-10.67302771101574,0.04284178809200095,-0.03196142272696709,6.240562907855309,-2.155834514035699,0.005686313649390949,-0.0557366891268529},
//{14.01269005282396,10.47231597547145,-0.007743097229952396,0.03092362736657574,6.239177445701783,2.127268613636207,0.005316980068840059,0.05510842988328495},
//{-0.001304310740079771,-0.03812349237943729,0.378024293325187,0.3252064219514601,-0.0002635211919329557,-0.003339767758877333,0.2610977588171128,0.2356684011756946},
//{-0.001002386412604174,0.0406471375369083,0.4434956274126712,-0.3154304780452504,-0.0001553768338996878,0.003904593443683988,0.2602312053407402,-0.2348975490079081} };

//Pitch mais rapido - bom
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02578348210831,-10.67055128744823,0.03798522260330547,-0.04613057429481957,6.240564890336407,-2.15547939574062,0.005725911859277205,-0.06576022205311377},
//{14.01268286509862,10.46988882049192,-0.003698524523851303,0.04484654574488396,6.239175088196358,2.126920697964934,0.005353550136059286,0.06494785408264969},
//{-0.001875333430842356,-0.06135044159459858,0.6199376760635573,0.4013241338134781,-0.000370218793769218,-0.006315329185710952,0.3257670608329968,0.2885273599412381},
//{-0.001549424750277614,0.06439176425174957,0.6809460466756787,-0.3884144222228286,-0.0002546475363179105,0.006999929604126315,0.3243909941599943,-0.2875297642940329} };

//Pitch mais rapido
//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02578296165945,-10.67055055347376,0.04125678749902133,-0.04613988509823993,6.240564745537068,-2.155479385698369,0.005901385853885981,-0.0657601438934762},
//{14.01268395206041,10.46988827348274,-0.007315772412376091,0.04486567469833183,6.239175377980488,2.126920803810915,0.005234333710028757,0.06494814118713103},
//{-0.001782324062838429,-0.06113766173104506,0.8210141148415369,0.4025656390517067,-0.0003537085690238672,-0.006268402979429174,0.3705925051502281,0.2886554699468938},
//{-0.001461211917964444,0.06460398753629101,0.8887418629602639,-0.3872688951465815,-0.0002394363496315402,0.007045991912361244,0.3690313939138746,-0.2874044720750278} };

//float32_t LQR_AH_Ke_f32[4][8]={
//{14.02909897733921,-10.84164029638927,0.04199692154853336,-0.02960482180030764,6.241468900929512,-1.683695760358056,0.006164200383100864,-0.05375063607422475},
//{14.00941877890456,10.64365487436031,-0.008772825704231047,0.028651773326234,6.238283593323756,1.664087029098413,0.005027990968745009,0.05316240215772602},
//{-0.002091756153764324,-0.05136594903112416,1.010802354492538,0.4442430761534243,-0.0004202682070790849,-0.001552810051168395,0.4412186349944942,0.3173990682570817},
//{-0.001718028168755959,0.05609243025693617,1.080195446097365,-0.4264838286003019,-0.0002818926382743317,0.002452241038236997,0.4397027100788979,-0.3158665464347218} };

//float32_t LQR_AH_Ke_f32[4][8]={
//{14.0257829826543,-10.67055058150586,0.04116900901018781,-0.04613960956454153,6.240564750650942,-2.155479385311288,0.005888444206386103,-0.06576015481330844},
//{14.01268392436012,10.46988829843457,-0.007206588450760501,0.04486530927852782,6.239175371403722,2.126920801816333,0.005246724152439353,0.0649481481261053},
//{-0.001783559612182664,-0.06114491247200617,0.8214430351547424,0.4025510974420015,-0.0003538243027100192,-0.006269652953822141,0.3690043722109584,0.2886518320246469},
//{-0.001462372944569921,0.06459673952262507,0.8889003678445405,-0.3872824249068383,-0.0002395352379776372,0.007044756028555338,0.3674306570777419,-0.287408089302814} };

float32_t LQR_AH_Ke_f32[4][8]={
{14.02578350951786,-10.67055131698765,0.03788442487421396,-0.04613021506000048,6.240564896872534,-2.155479394403389,0.005712881114574538,-0.06576023071469568},
{14.01268282872963,10.46988884676021,-0.003571628715816836,0.04484606763905028,6.239175079707442,2.126920694592634,0.005365951119588141,0.06494785772162551},
{-0.001876896091179444,-0.06135907287138962,0.6204180948022436,0.4013063542117242,-0.0003703637103762744,-0.006316854934856455,0.3239511045370437,0.2885230894415984},
{-0.00155088988810874,0.06438313564362241,0.6811088393232131,-0.3884308897834236,-0.0002547705622286386,0.006998422054245339,0.3225629277922851,-0.2875340041052385} };

float32_t LQR_PT_Ke_f32[4][16]={
{-0.000509474023994 ,  1.381002006541810 ,  2.044930990723325 , -4.098388643419657 ,  0.002544968427177 ,  0.065243786421189, -0.011997162152724 ,  0.012231188237446 , -0.000191109567030 ,  0.977046060078436  , 2.067519443836474 , -1.069820095142832,
    0.003981280957280 ,  0.048837474399233 , -0.000157117358773  , 0.000160465562387},
{0.000485844262540 , -1.379777079391250 ,  2.046778453514912 ,  4.095526259295323 ,  0.006483505671600 , -0.065184427882726,
    0.011988462007403 , -0.012218371367099 , 0.000647666101473 , -0.976219155907380 ,  2.069372953111319  , 1.069647999496566,
    0.005073107886213 , -0.048792319825525 ,  0.000156410506717 , -0.000160891309965},
{0.147511637570957 , -0.044700625197703  , 0.000027359219549  , 0.100038920576059  , 0.186220549413359 ,  0.044874312445416,
    0.218427852703492 ,  0.036876304589717 ,  0.070402232740872 , -0.028416542202667 ,  0.000007473706141 ,  0.017792742949581,
    0.025919243351085 ,  0.030976534992245 ,  0.007155474366137 ,  0.000723596619702},
{0.147480373299421 ,  0.044731365984124 ,  0.000026937597506 , -0.099883806663005 ,  0.186155931716280 , -0.044878136021767,
    0.036878957916608 ,  0.218443336829840 ,  0.070383903601564 ,  0.028402976825487  , 0.000007312089313 , -0.017740572220967,
    0.025908596148404,  -0.030981560936545  , 0.000723658016145  , 0.007155720063258} };

float32_t equilibrium_point_f32[] ;
float32_t equilibrium_control_f32[] ;
float32_t state_vector_f32[] ;
float32_t error_state_vector_f32[] ;

#ifdef LQR_ATTITUDE_HEIGHT_CONTROL
float32_t equilibrium_point_f32[8]={0, 0, -0.079091, 0, 0, 0, 0, 0};
//float32_t equilibrium_point_f32[8]={0, 0.8913f,-0.0791f,-1.123453f, 0, 0, 0, 0};
//float32_t equilibrium_control_f32[8]={8.5294, 8.5524, 0.079189, 0.078994};
float32_t equilibrium_control_f32[8]={0.113152E2,0.113459E2,0.79091E-1,0.79091E-1};
float32_t state_vector_f32[8]={0};
float32_t error_state_vector_f32[8]={0};

#elif defined LQR_PATHTRACK_CONTROL

float32_t equilibrium_control_f32[4]={10.2751,10.2799,-0.0154,-0.0154};
float32_t equilibrium_point_f32[16]={0};
float32_t state_vector_f32[16]={0};
float32_t error_state_vector_f32[16]={0};

#endif


float32_t PD_control_output_f32[4]={0};
float32_t I_control_output_f32[4]={0};
float32_t PID_control_output_f32[4]={0};
float32_t control_output_f32[4]={0};
float32_t  integrated_error_states_f32[4]={0};

arm_matrix_instance_f32 equilibrium_control;
arm_matrix_instance_f32 LQR_AH_Ke;
arm_matrix_instance_f32 LQR_AH_Ki;

arm_matrix_instance_f32 LQR_PT_Ke;
arm_matrix_instance_f32 LQR_PT_Ki;
arm_matrix_instance_f32 LQR_PT_FF;


/* Private function prototypes -----------------------------------------------*/
/* LQR attitude and height(AH) controller */
arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, pv_type_datapr_position position, pv_type_datapr_position position_reference);
arm_matrix_instance_f32 c_rc_LQR_AH_PD(arm_matrix_instance_f32 error_state_vector);
arm_matrix_instance_f32 c_rc_LQR_AH_I(pv_type_stability_error error);
void c_rc_LQR_AH_integrate_error(pv_type_stability_error current_error, float sample_time);

/*//LQR Path Track (PT) controller.*/
arm_matrix_instance_f32 c_rc_LQR_PT_errorStateVector(pv_msg_input msg_input ) ;

void c_rc_LQR_PT_integrate_error(pv_type_pathtrack_error error, float sample_time) ;
arm_matrix_instance_f32 c_rc_LQR_PT_PD(arm_matrix_instance_f32 error_state_vector);
arm_matrix_instance_f32 c_rc_LQR_PT_I(pv_type_pathtrack_error error, bool enable_integration) ;
void c_RC_LQR_PT_StepRef(pv_msg_input *msg_input) ;

/* Private functions ---------------------------------------------------------*/

void c_rc_LQR_AH_integrate_error(pv_type_stability_error error, float sample_time){

	#ifdef ENABLE_INT_ROLL
		LQR_AH_integrated_error.roll = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.roll, error.roll,
				LQR_AH_last_error.roll, sample_time), INT_ROLL_LOWER_ER_LIMIT, INT_ROLL_UPPER_ER_LIMIT);

		LQR_AH_last_error.roll  = error.roll;
	#endif
	#ifdef ENABLE_INT_PITCH
		LQR_AH_integrated_error.pitch = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.pitch, error.pitch,
				LQR_AH_last_error.pitch, sample_time), INT_PITCH_LOWER_ER_LIMIT, INT_PITCH_UPPER_ER_LIMIT);

		LQR_AH_last_error.pitch = error.pitch;
	#endif
	#ifdef ENABLE_INT_YAW
		LQR_AH_integrated_error.yaw = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.yaw, error.yaw,
				LQR_AH_last_error.yaw, sample_time), INT_YAW_LOWER_ER_LIMIT, INT_YAW_UPPER_ER_LIMIT);

		LQR_AH_last_error.yaw   = error.yaw;
	#endif
	#ifdef ENABLE_INT_Z
		LQR_AH_integrated_error.z = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_AH_integrated_error.z, error.z,
				LQR_AH_last_error.z, sample_time), INT_Z_LOWER_ER_LIMIT, INT_Z_UPPER_ER_LIMIT);

		LQR_AH_last_error.z   = error.z;
	#endif
}



arm_matrix_instance_f32 c_rc_LQR_AH_errorStateVector(pv_type_datapr_attitude attitude,
                                                     pv_type_datapr_attitude attitude_reference,
                                                     pv_type_datapr_position position,
                                                     pv_type_datapr_position position_reference)
{

	arm_matrix_instance_f32 error_state_vector, state_vector, equilibrium_point;

	//State Vector
	state_vector_f32[STATE_Z]=position.z;
	state_vector_f32[STATE_ROLL]=attitude.roll;
	state_vector_f32[STATE_PITCH]=attitude.pitch;
	state_vector_f32[STATE_YAW]=attitude.yaw;
	state_vector_f32[STATE_DZ]=position.dotZ;
	state_vector_f32[STATE_DROLL]=attitude.dotRoll;
	state_vector_f32[STATE_DPITCH]=attitude.dotPitch;
	state_vector_f32[STATE_DYAW]=attitude.dotYaw;

	//Updates the height equilibrium point according to the reference
	equilibrium_point_f32[STATE_Z]= position_reference.z;
	equilibrium_point_f32[STATE_DZ]= position_reference.dotZ;
	equilibrium_point_f32[STATE_ROLL]= attitude_reference.roll;
	equilibrium_point_f32[STATE_PITCH]= attitude_reference.pitch;

	//Initializes the matrices
	arm_mat_init_f32(&equilibrium_point, 8, 1, (float32_t *)equilibrium_point_f32);
	arm_mat_init_f32(&state_vector, 8, 1, (float32_t *)state_vector_f32);
	arm_mat_init_f32(&error_state_vector, 8, 1, (float32_t *)error_state_vector_f32);
	//e(t)=x(t)- equilibrium_point
	arm_mat_sub_f32(&state_vector, &equilibrium_point, &error_state_vector);

	return error_state_vector;
}



arm_matrix_instance_f32 c_rc_LQR_AH_PD(arm_matrix_instance_f32 error_state_vector){

	arm_matrix_instance_f32 PD_control_output;


	//Initializing result matrices
	arm_mat_init_f32(&PD_control_output, 4, 1, (float32_t *)PD_control_output_f32);
	//u=-Ke*e(t)
	arm_mat_mult_f32(&LQR_AH_Ke, &error_state_vector, &PD_control_output);

	return PD_control_output;
}



arm_matrix_instance_f32 c_rc_LQR_AH_I(pv_type_stability_error error){

	arm_matrix_instance_f32 I_control_output, integrated_error_states;


	//Initializing result matrix
	arm_mat_init_f32(&I_control_output, 4, 1, (float32_t *)I_control_output_f32);
	// Integrate the error
	c_rc_LQR_AH_integrate_error(error, CONTROL_SAMPLE_TIME);
	//Create the integrated error vector
	integrated_error_states_f32[STATE_ROLL] =	LQR_AH_integrated_error.roll;
	integrated_error_states_f32[STATE_PITCH]= 	LQR_AH_integrated_error.pitch;
	integrated_error_states_f32[STATE_YAW]=	 	LQR_AH_integrated_error.yaw;
	integrated_error_states_f32[STATE_Z]=	 	LQR_AH_integrated_error.z;
	arm_mat_init_f32(&integrated_error_states, 4, 1, (float32_t *)integrated_error_states_f32);
	//u=Ki*e(t)
	arm_mat_mult_f32(&LQR_AH_Ki, &integrated_error_states, &I_control_output);

	return I_control_output;
}

arm_matrix_instance_f32 c_rc_LQR_PT_errorStateVector(pv_msg_input msg_input )
		/*pv_type_datapr_attitude attitude,
                                                     pv_type_datapr_attitude attitude_reference,
                                                     pv_type_datapr_position position,
                                                     pv_type_datapr_position position_reference,
                                                     pv_type_datapr_servos servo_state)*/
{

  arm_matrix_instance_f32 error_state_vector, state_vector, equilibrium_point;

  //State Vector
  state_vector_f32[STATE_X]=msg_input.position.x;
  state_vector_f32[STATE_Y]=msg_input.position.y;
  state_vector_f32[STATE_Z]=msg_input.position.z;
  state_vector_f32[STATE_ROLL]=msg_input.attitude.roll;
  state_vector_f32[STATE_PITCH]=msg_input.attitude.pitch;
  state_vector_f32[STATE_YAW]=msg_input.attitude.yaw;
  state_vector_f32[STATE_ALPHA_R]=msg_input.servosOutput.servo.alphar;
  state_vector_f32[STATE_ALPHA_L]=msg_input.servosOutput.servo.alphal;
  state_vector_f32[STATE_DX]=msg_input.position.dotX;
  state_vector_f32[STATE_DY]=msg_input.position.dotY;
  state_vector_f32[STATE_DZ]=msg_input.position.dotZ;
  state_vector_f32[STATE_DROLL]=msg_input.attitude.dotRoll;
  state_vector_f32[STATE_DPITCH]=msg_input.attitude.dotPitch;
  state_vector_f32[STATE_DYAW]=msg_input.attitude.dotYaw;
  state_vector_f32[STATE_DALPHA_R]=msg_input.servosOutput.servo.dotAlphar;
  state_vector_f32[STATE_DALPHA_L]=msg_input.servosOutput.servo.dotAlphal;



  //Updates the height equilibrium point according to the reference
  equilibrium_point_f32[STATE_X]= msg_input.position_reference.x;
  equilibrium_point_f32[STATE_Y]= msg_input.position_reference.y;
  equilibrium_point_f32[STATE_Z]= msg_input.position_reference.z;
  equilibrium_point_f32[STATE_YAW]= msg_input.attitude_reference.yaw;

  //Initializes the matrices
  arm_mat_init_f32(&equilibrium_point, 16, 1, (float32_t *)equilibrium_point_f32);
  arm_mat_init_f32(&state_vector, 16, 1, (float32_t *)state_vector_f32);
  arm_mat_init_f32(&error_state_vector, 16, 1, (float32_t *)error_state_vector_f32);

  //e(t)=x(t)- equilibrium_point
  arm_mat_sub_f32(&state_vector, &equilibrium_point, &error_state_vector);

  return error_state_vector;
}

void c_rc_LQR_PT_integrate_error(pv_type_pathtrack_error error, float sample_time){
  #ifdef ENABLE_INT_X
    //LQR_PT_integrated_error.x = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_PT_integrated_error.x, error.x,
                                                                          //  LQR_PT_last_error.x, sample_time), INT_X_LOWER_ER_LIMIT, INT_X_UPPER_ER_LIMIT);
    LQR_PT_integrated_error.x=c_rc_integrate_trapezoidal(LQR_PT_integrated_error.x, error.x,LQR_PT_last_error.x, sample_time) ;
    LQR_PT_last_error.x   = error.x;
  #endif

  #ifdef ENABLE_INT_Y
  //LQR_PT_integrated_error.y = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_PT_integrated_error.y, error.y,
                                                                          //  LQR_PT_last_error.y, sample_time), INT_Y_LOWER_ER_LIMIT, INT_Y_UPPER_ER_LIMIT);
    LQR_PT_integrated_error.y=c_rc_integrate_trapezoidal(LQR_PT_integrated_error.y, error.y,LQR_PT_last_error.y, sample_time) ;
    LQR_PT_last_error.y   = error.y;
  #endif
  #ifdef ENABLE_INT_Z
    //LQR_PT_integrated_error.z = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_PT_integrated_error.z, error.z,
                                                                          //  LQR_PT_last_error.z, sample_time), INT_Z_LOWER_ER_LIMIT, INT_Z_UPPER_ER_LIMIT);
    LQR_PT_integrated_error.z=c_rc_integrate_trapezoidal(LQR_PT_integrated_error.z, error.z,LQR_PT_last_error.z, sample_time) ;
    LQR_PT_last_error.z   = error.z;
  #endif

  #ifdef ENABLE_INT_YAW
    //LQR_PT_integrated_error.yaw = c_rc_saturation( c_rc_integrate_trapezoidal(LQR_PT_integrated_error.yaw, error.yaw,
                                                                             // LQR_PT_last_error.yaw, sample_time), INT_YAW_LOWER_ER_LIMIT, INT_YAW_UPPER_ER_LIMIT);
    LQR_PT_integrated_error.yaw=c_rc_integrate_trapezoidal(LQR_PT_integrated_error.yaw, error.yaw,LQR_PT_last_error.yaw, sample_time) ;
    LQR_PT_last_error.yaw   = error.yaw;
  #endif

}

arm_matrix_instance_f32 c_rc_LQR_PT_PD(arm_matrix_instance_f32 error_state_vector){

  arm_matrix_instance_f32 PD_control_output;


  //Initializing result matrices
  arm_mat_init_f32(&PD_control_output, 4, 1, (float32_t *)PD_control_output_f32);
  //u=-Ke*e(t)
  arm_mat_mult_f32(&LQR_PT_Ke, &error_state_vector, &PD_control_output);

  return PD_control_output;
}

arm_matrix_instance_f32 c_rc_LQR_PT_I(pv_type_pathtrack_error error, bool enable_integration){

  arm_matrix_instance_f32 I_control_output, integrated_error_states;


  //Initializing result matrix
  arm_mat_init_f32(&I_control_output, 4, 1, (float32_t *)I_control_output_f32);


  // Integrate the error
  if (enable_integration)
    c_rc_LQR_PT_integrate_error(error, CONTROL_SAMPLE_TIME );

  //Create the integrated error vector
  integrated_error_states_f32[STATE_X] = LQR_PT_integrated_error.x;
  integrated_error_states_f32[STATE_Y]=   LQR_PT_integrated_error.y;
  integrated_error_states_f32[STATE_Z]=   LQR_PT_integrated_error.z;
  integrated_error_states_f32[STATE_YAW-2]=   LQR_PT_integrated_error.yaw; // integrated_error_states size = 4
  arm_mat_init_f32(&integrated_error_states, 4, 1, (float32_t *)integrated_error_states_f32);
  //u=Ki*e(t)
  arm_mat_mult_f32(&LQR_PT_Ki, &integrated_error_states, &I_control_output);

  return I_control_output;
}

void c_RC_LQR_PT_StepRef(pv_msg_input *msg_input){


  float reference[4]={msg_input->position_reference.x,msg_input->position_reference.y,msg_input->position_reference.z,msg_input->attitude_reference.yaw} ;
  float state[4]={msg_input->position.x,msg_input->position.y,msg_input->position.z,msg_input->attitude.yaw} ;
  float new_reference[4]={0} ;
  float distance[4]={0} ;


  for (int i=0;i<4;i++){
    distance[i] = reference[i] - state[i] ;


    if (fabs(distance[i])>stepSize[i]) {
      if (distance[i]>0){
        new_reference[i]=state[i] +stepSize[i] ;
      }
      else {
        new_reference[i]=state[i]-stepSize[i] ;
      }
    }
    else {
      new_reference[i]=reference[i] ;
    }
  }
  msg_input->position_reference.x = new_reference[0] ;
  msg_input->position_reference.y = new_reference[1] ;
  msg_input->position_reference.z = new_reference[2] ;
  msg_input->attitude_reference.yaw = new_reference[3] ;


}


/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_rc_LQR_control_init() {

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&equilibrium_control, 4, 1, (float32_t *)equilibrium_control_f32);

#ifdef LQR_ATTITUDE_HEIGHT_CONTROL
	arm_mat_init_f32(&LQR_AH_Ke, 4, 8, (float32_t *)LQR_AH_Ke_f32);
	arm_mat_init_f32(&LQR_AH_Ki, 4, 4, (float32_t *)LQR_AH_Ki_f32);
#endif

#ifdef LQR_PATHTRACK_CONTROL
  arm_mat_init_f32(&LQR_PT_Ke, 4, 16, (float32_t *)LQR_PT_Ke_f32);
  arm_mat_init_f32(&LQR_PT_Ki, 4, 4, (float32_t *)LQR_PT_Ki_f32);

#endif

}



/* \brief LQR Controller.
 *
 * Implemented based on the article "Back-stepping Control Strategy for Stabilization of a Tilt-rotor UAV" by Chowdhury, A. B., with some modifications.
 * It implements an estabilization controller and an altitude controller. It is meant to be used with the radio controller.
 * The struct pv_msg_io_attitude includes the angular velocity.
 */
const float height_takeoff_a=-1.2;
const float height_takeoff_b=-0.5;
const float height_takeoff_c=1.2;
float total_time_height_control=0;
float last_reference_z=0;


pv_type_actuation c_rc_LQR_AH_controller(pv_type_datapr_attitude attitude,
				  pv_type_datapr_attitude attitude_reference,
				  pv_type_datapr_position position,
				  pv_type_datapr_position position_reference,
				  float throttle_control,
				  bool manual_height_control){

	pv_type_actuation actuation_signals;
	arm_matrix_instance_f32 error_state_vector, PD_control, I_control, PID_control, control_output;
	pv_type_stability_error error;
	float temp_height_takeoff=0;


	//Initialize result matrices
	arm_mat_init_f32(&PID_control, 4, 1, (float32_t *)PID_control_output_f32);
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);

	//Updates the height equilibrium point according to the reference
	#ifdef ENABLE_TAKEOFF_PROCEDURE
		if(!manual_height_control){
			temp_height_takeoff=height_takeoff_a*exp(height_takeoff_b*total_time_height_control);
			position_reference.z=temp_height_takeoff+height_takeoff_c;
			position_reference.dotZ=height_takeoff_b*temp_height_takeoff;

			total_time_height_control += CONTROL_SAMPLE_TIME;}
		else
			total_time_height_control=0;
	#endif


	error_state_vector = c_rc_LQR_AH_errorStateVector(attitude, attitude_reference, position, position_reference);

	//u_p+u_d=-Ke*e(t)
	PD_control = c_rc_LQR_AH_PD(error_state_vector);
	//u_i=-Ki*integral(e(t))
	//Select variables to integrate( same as ref - C*x(t) )
//	error.roll=	 attitude_reference.roll-attitude.roll;
//	error.pitch= attitude_reference.pitch-attitude.pitch;
//	error.yaw=	 attitude_reference.yaw-attitude.yaw;
//	error.z=	 position_reference.z-position.z;
//	I_control = c_rc_LQR_AH_I(error);
	//u_pid=u_pd+u_i;
//	arm_mat_add_f32(&PD_control, &I_control, &PID_control);
	//u(t)=u_pid+u_r
//	arm_mat_sub_f32(&equilibrium_control, &PID_control, &control_output);
	arm_mat_sub_f32(&equilibrium_control, &PD_control, &control_output);
	//The result must be in a struct pv_msg_io_actuation
	actuation_signals.escRightSpeed= (float)control_output.pData[0];
	actuation_signals.escLeftSpeed=	 (float)control_output.pData[1];
	actuation_signals.servoRight=	 (float)control_output.pData[2];
	actuation_signals.servoLeft=	 (float)control_output.pData[3];
    //Declares that the servos will use angle control, rather than torque control
	actuation_signals.servoTorqueControlEnable = 0;

	return actuation_signals;
}

pv_type_actuation c_rc_LQR_PT_controller(pv_msg_input msg_input){

  pv_type_datapr_attitude attitude = msg_input.attitude ;
  pv_type_datapr_attitude attitude_reference = msg_input.attitude_reference ;
  pv_type_datapr_position position= msg_input.position ;
  pv_type_datapr_position position_reference= msg_input.position_reference ;
  pv_type_datapr_servos servo_state = msg_input.servosOutput.servo ;


  pv_type_actuation actuation_signals;
  arm_matrix_instance_f32 error_state_vector, PD_control, I_control, PID_control, control_output;
  pv_type_pathtrack_error error;

  //Initialize result matrices
  arm_mat_init_f32(&PID_control, 4, 1, (float32_t *)PID_control_output_f32);
  arm_mat_init_f32(&error_state_vector, 4, 1, (float32_t *)error_state_vector_f32);
  arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);

  c_RC_LQR_PT_StepRef( &msg_input) ; // adapt the reference if it is too big

  // e = X -Xref
  error_state_vector = c_rc_LQR_PT_errorStateVector(msg_input);
    //u_p+u_d=-Ke*e(t)
  PD_control = c_rc_LQR_PT_PD(error_state_vector);

    //u_i=-Ki*integral(e(t))
    //Select variables to integrate( same as ref - C*x(t) )
  error.x=   error_state_vector_f32[STATE_X];
  error.y=   error_state_vector_f32[STATE_Y];
  error.z=   error_state_vector_f32[STATE_Z];
  error.yaw=   error_state_vector_f32[STATE_YAW];

  I_control = c_rc_LQR_PT_I(error,msg_input.enableintegration);

    //u_pid=u_pd+u_i;
  arm_mat_add_f32(&PD_control, &I_control, &PID_control);

    //u(t)=u_pid+u_r
  arm_mat_sub_f32(&equilibrium_control, &PID_control, &control_output);
   //arm_mat_sub_f32(&equilibrium_control, &PD_control, &control_output);

    //The result must be in a struct pv_msg_io_actuation
    actuation_signals.escRightNewtons= (float)control_output.pData[0];
    actuation_signals.escLeftNewtons=  (float)control_output.pData[1];
    actuation_signals.servoRight=  (float)control_output.pData[2];
    actuation_signals.servoLeft=   (float)control_output.pData[3];


      //Declares that the servos will use angle control, rather than torque control
    actuation_signals.servoTorqueControlEnable = 0;



    return actuation_signals;
}


/* IRQ handlers ------------------------------------------------------------- */

/**
  * @}
  */

/**
  * @}
  */

