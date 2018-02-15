/*
 * c_rc_ref_generation.h
 *
 *  Created on: Feb 8, 2018
 *      Author: provant
 */

#ifndef C_RC_REF_GENERATION_H_
#define C_RC_REF_GENERATION_H_

#define ARM_MATH_CM4
#include "pv_typedefs.h"
#include "arm_math.h"
#include "c_rc_commons.h"

#define points_number 2

float x_points[points_number] ;
float y_points[points_number] ;
float z_points[points_number] ;
float yaw_points[points_number] ;

void c_rc_ref_init()  ;
void c_rc_ref_discrete(pv_msg_input *input) ;
void c_rc_ref_continuous(pv_msg_input *input) ;


#endif /* C_RC_REF_GENERATION_H_ */
