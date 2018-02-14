/*
 * c_rc_ref_generation.c
 *
 *  Created on: Feb 8, 2018
 *      Author: provant
 */
#include "c_rc_ref_generation.h"



void c_rc_ref_init() {
	x_points[0] = 0 ;
	y_points[0] = 0 ;
	z_points[0] = 0.5 ;


}

void c_rc_ref_discrete(pv_msg_input *input){
  static float startDelay = 0 ;
  static int countPoint = 0 ;
  int count = 0 ;
  float xErr = fabs(input->position.x - input->position_reference.x) ;
  float yErr = fabs(input->position.y - input->position_reference.y) ;
  float zErr = fabs(input->position.z - input->position_reference.z) ;
  float ErrSum = xErr + yErr + zErr ;

  if ( fabs(ErrSum)<0.05 ) {
    count++ ;


    if (count>1200 ) {
      if (countPoint<points_number-1){
        countPoint++ ;
        input->position_reference.x =  x_points[countPoint] ;
        input->position_reference.y =  y_points[countPoint];
        input->position_reference.z =  z_points[countPoint] ;
      }
    }
  }
  else
    count=0 ;
}
