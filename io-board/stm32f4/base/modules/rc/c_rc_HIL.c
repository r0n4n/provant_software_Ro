/**
  ******************************************************************************
  * @file    modules/rc/c_rc_HIL.c
  * @author  Ronan Blanchard
  * @version V1.0.0
  * @date    Feb 3, 2018
  * @brief	update the pv_msg_input with the data received from the uart
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_rc_HIL.h"


void c_rc_set_state(float state[], pv_msg_input *InputData ){

#ifdef LQR_PATHTRACK_CONTROL
  InputData->position.x = state[STATE_X];
  InputData->position.y = state[STATE_Y];
  InputData->position.z = state[STATE_Z];
  InputData->position.dotX = state[STATE_DX] ;
  InputData->position.dotY = state [STATE_DY] ;
  InputData->position.dotZ = state[STATE_DZ] ;

  // get servo motor position
  InputData->servosOutput.servo.alphar=state[STATE_ALPHA_R];
  InputData->servosOutput.servo.alphal=state[STATE_ALPHA_L];
  InputData->servosOutput.servo.dotAlphar=state[STATE_DALPHA_R];
  InputData->servosOutput.servo.dotAlphal=state[STATE_DALPHA_L];

  /* get the attitude*/
  InputData->attitude.roll  = state[STATE_ROLL];
  InputData->attitude.pitch = state[STATE_PITCH];
  InputData->attitude.yaw   = state[STATE_YAW];
  InputData->attitude.dotRoll  = state[STATE_DROLL];
  InputData->attitude.dotPitch = state[STATE_DPITCH];
  InputData->attitude.dotYaw   = state[STATE_DYAW];

#elif defined HINF_PATHTRACK_CONTROL
  InputData->position.x = state[STATE_X];
  InputData->position.y = state[STATE_Y];
  InputData->position.z = state[STATE_Z];
  InputData->position.dotX = state[STATE_DX] ;
  InputData->position.dotY = state [STATE_DY] ;
  InputData->position.dotZ = state[STATE_DZ] ;

  // get servo motor position
  InputData->servosOutput.servo.alphar=state[STATE_ALPHA_R];
  InputData->servosOutput.servo.alphal=state[STATE_ALPHA_L];
  InputData->servosOutput.servo.dotAlphar=state[STATE_DALPHA_R];
  InputData->servosOutput.servo.dotAlphal=state[STATE_DALPHA_L];

  /* get the attitude*/
  InputData->attitude.roll  = state[STATE_ROLL];
  InputData->attitude.pitch = state[STATE_PITCH];
  InputData->attitude.yaw   = state[STATE_YAW];
  InputData->attitude.dotRoll  = state[STATE_DROLL];
  InputData->attitude.dotPitch = state[STATE_DPITCH];
  InputData->attitude.dotYaw   = state[STATE_DYAW];

  /*GET THE LOAD ATTITUDE */
  InputData->load_attitude.x_angle = state[STATE_LOAD_X_ANGLE] ;
  InputData->load_attitude.y_angle = state[STATE_LOAD_Y_ANGLE] ;
  InputData->load_attitude.dotx_angle = state[STATE_LOAD_X_ANGLE_VELOCITY] ;
  InputData->load_attitude.doty_angle = state[STATE_LOAD_Y_ANGLE_VELOCITY] ;


#endif


}
