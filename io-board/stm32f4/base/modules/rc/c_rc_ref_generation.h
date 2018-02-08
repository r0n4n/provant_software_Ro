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

#define points_number 2

float x_points[points_number] = {0,1} ;


void c_rc_discrete_ref(pv_msg_input *input) ;

#endif /* C_RC_REF_GENERATION_H_ */
