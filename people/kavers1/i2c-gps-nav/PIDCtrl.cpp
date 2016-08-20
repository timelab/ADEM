#include <math.h>
#include "PIDCtrl.h"

float PIDCtrl::_filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )"
int32_t PIDCtrl::get_p(int32_t error)
{
	return (float)error * _kp;
}

int32_t PIDCtrl::get_i(int32_t error, float dt)
{
	if((_ki != 0) && (dt != 0)){
		_integrator += ((float)error * _ki) * dt;
		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
		return _integrator;
	}
	return 0;
}

int32_t PIDCtrl::get_d(int32_t input, float dt)
{
	if ((_kd != 0) && (dt != 0)) {
		_derivative = (input - _last_input) / dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = _last_derivative +
		        (dt / ( _filter + dt)) * (_derivative - _last_derivative);

		// update state
		_last_input 		= input;
		_last_derivative    = _derivative;

		// add in derivative component
		return _kd * _derivative;
	}
	return 0;
}

int32_t PIDCtrl::get_pi(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt);
}


int32_t PIDCtrl::get_pid(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt) + get_d(error, dt);
}

void
PIDCtrl::reset_I()
{
	_integrator = 0;
	_last_input = 0;
	_last_derivative = 0;
}
