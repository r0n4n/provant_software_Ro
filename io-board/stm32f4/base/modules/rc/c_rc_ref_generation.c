/*
 * c_rc_ref_generation.c
 *
 *  Created on: Feb 8, 2018
 *      Author: provant
 */
#include "c_rc_ref_generation.h"

float trajectoryRadius = 2;
//float trajectoryHeight = 4*trajectoryRadius;
float trajTime = 30;
float pi = 3.14;

void c_rc_ref_init() {
	x_points[0] = 1 ;
	y_points[0] = 0 ;
	z_points[0] = 1;
	yaw_points[0] = 0;

	x_points[1] = 0.5 ;
	y_points[1] = 0.5 ;
	z_points[1] = 0.5 ;
	yaw_points[1] = 0;


}

void c_rc_ref_discrete(pv_msg_input *input){
	static float startDelay = 0 ;
	static int countPoint = 0 ;
	static int count = 0 ;
	if (countPoint<points_number-1){

		float xErr = fabs(input->position.x - input->position_reference.x) ;
		float yErr = fabs(input->position.y - input->position_reference.y) ;
		float zErr = fabs(input->position.z - input->position_reference.z) ;
		float yawErr = fabs(input->attitude.yaw - input->attitude_reference.yaw) ;
		float ErrSum = xErr + yErr + zErr + yawErr  ;

		if ( fabs(ErrSum)<0.05 ) {
			count++ ;
			if (count>10 ) {
				countPoint++ ;
				input->position_reference.x =  x_points[countPoint] ;
				input->position_reference.y =  y_points[countPoint];
				input->position_reference.z =  z_points[countPoint] ;
				input->attitude_reference.yaw =  yaw_points[countPoint] ;
			}
		}
		else
			count=0 ;
	}
}

void c_rc_ref_continuous(pv_msg_input *input){


		static int count = 0 ;
		count++ ;

		// linear trajectory generation
		input->position_reference.x = trajectoryRadius*cos((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);
		//input->position_reference.dotX = -trajectoryRadius*(2*pi/trajTime)*sin((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);

		input->position_reference.y = trajectoryRadius*sin((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);
		//input->position_reference.dotY = trajectoryRadius*(2*pi/trajTime)*cos((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);

		//input->position_reference.z = trajectoryHeight+1 - trajectoryHeight*cos((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);
		//input->position_reference.dotZ = trajectoryHeight*(2*pi/trajTime)*sin((count*CONTROL_SAMPLE_TIME)*2*pi/trajTime);

}
