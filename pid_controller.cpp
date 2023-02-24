/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->Kp = Kpi;
  this->Ki = Kii;
  this->Kd = Kdi;
  this->output_lim_max = output_lim_maxi;
  this->output_lim_min = output_lim_mini;
  this->total_cte = 0.0;
  this->cte = 0.0;
  this->delta_time = 1.0;
}


void PID::UpdateError(double cte) {
  this->prev_cte = this->cte;
  this->cte = cte;
  this->total_cte += this->cte*this->delta_time;
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control;
  	
    //std::cout<<"delta time"<<this->delta_time<<std::endl;
  	if(delta_time < 0.001) delta_time = 1.0;
  	
  	double d_control = Kd*(cte - prev_cte)/delta_time;//derivative conrol
  	double p_control = Kp*cte;//proportional control  
  	double i_control = Ki*(total_cte);//integral control
  	
  	// uncomment to reduce drastic effects of derviative part
  	//if(d_control>= output_lim_max/2) d_control= output_lim_max/2;
  	//if(d_control<= output_lim_min/2) d_control= output_lim_min/2;
  
  	control = p_control + i_control + d_control; //output of PID controller 
  
  	if(control > output_lim_max) return output_lim_max;
  	if(control < output_lim_min) return output_lim_min;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  	
  	this->delta_time = new_delta_time;
   /**
   * TODO: Update the delta time with new value
   */
}
