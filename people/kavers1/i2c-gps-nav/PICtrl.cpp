#include <math.h>
#include "PICtrl.h"

int32_t PICtrl::get_p(int32_t error)
{
	return (float)error * _kp;
}

int32_t PICtrl::get_i(int32_t error, float dt)
{
	if(dt != 0){
		_integrator += ((float)error * _ki) * dt;

		if (_integrator < -_imax) {
			_integrator = -_imax;
		} else if (_integrator > _imax) {
			_integrator = _imax;
		}
	}
	return _integrator;
}

int32_t PICtrl::get_pi(int32_t error, float dt)
{
	return get_p(error) + get_i(error, dt);
}

void
PICtrl::reset_I()
{
	_integrator = 0;
}
