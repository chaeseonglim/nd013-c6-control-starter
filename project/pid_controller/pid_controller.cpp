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
  
   _cte_p = 0.0;
   _cte_i = 0.0;
   _cte_d = 0.0;
   _kpi = Kpi;
   _kii = Kii;
   _kdi = Kdi;
   _output_lim_maxi = output_lim_maxi;
   _output_lim_mini = output_lim_mini;

}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  
   // _cte_d is derivative of cte
   _cte_d = ( _delta_time > 0 )? ( ( cte - _cte_p ) / _delta_time ) : 0.0;
    // _cte_p is identical to cte
   _cte_p = cte;
   // _cte_i is integral of cte
   _cte_i = _cte_i + cte * _delta_time;
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = ( - ( _cte_p * _kpi ) - ( _cte_i * _kii ) - ( _cte_d * _kdi ) );
  
    // Clip a value in range [_output_lim_mini, _output_lim_maxi]
    return std::min( _output_lim_maxi, std::max( _output_lim_mini, control ) );
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  _delta_time = new_delta_time;
  
  return _delta_time;
}