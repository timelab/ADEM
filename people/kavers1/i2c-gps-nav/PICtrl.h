#ifndef PICtrl_h
#define PICtrl_h

#include "Arduino.h"
#undef abs
#undef round
#include <stdint.h>
#include <math.h>		// for fabs()


/// @class	PICtrl
/// @brief	Object managing one PI control
class PICtrl {
public:

	/// Constructor for PI that saves its settings to EEPROM
	///
	/// @note	PI must be named to avoid either multiple parameters with the
	///			same name, or an overly complex constructor.
	///
	/// @param  initial_p       Initial value for the P term.
    /// @param  initial_i       Initial value for the I term.
    /// @param  initial_imax    Initial value for the imax term.4
	///
	PICtrl(const float &initial_p = 0.0,
		   const float &initial_i = 0.0,
		   const int16_t &initial_imax = 0.0) :
		_kp  (initial_p),
		_ki  (initial_i),
		_imax(initial_imax)
	{
		// no need for explicit load, assuming that the main code uses AP_Var::load_all.
	}

	/// Iterate the PI, return the new control value
	///
	/// Positive error produces positive output.
	///
	/// @param error	The measured error value
	/// @param dt		The time delta in milliseconds (note
	///					that update interval cannot be more
	///					than 65.535 seconds due to limited range
	///					of the data type).
	/// @param scaler	An arbitrary scale factor
	///
	/// @returns		The updated control output.
	///
	//long 	get_pi(int32_t error, float	 dt);
	int32_t get_pi(int32_t error, float dt);
	int32_t get_p(int32_t error);
	int32_t get_i(int32_t error, float dt);


	/// Reset the PI integrator
	///
	void	reset_I();


	/// @name	parameter accessors
	//@{

	/// Overload the function call operator to permit relatively easy initialisation
	void operator() (const float p,
	                 const float i,
	                 const int16_t imaxval) {
		_kp = p; _ki = i; _imax = imaxval;
	}

	float	kP() const				{ return _kp; }
	float	kI() const 				{ return _ki; }
	int16_t	imax() const			{ return _imax; }

	void	kP(const float v)		{ _kp=v; }
	void	kI(const float v)		{ _ki=v; }
	void	imax(const int16_t v)	{ _imax=abs(v); }
	float	get_integrator() const	{ return _integrator; }
	void	set_integrator(float i)	{ _integrator = i; }


private:
	float			_kp;
	float			_ki;
	int16_t			_imax;

	float				_integrator;		///< integrator value
};

#endif
