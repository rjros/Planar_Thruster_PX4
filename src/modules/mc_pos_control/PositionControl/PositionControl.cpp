/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>

#include <geo/geo.h>

using namespace matrix;

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}
//Planar Gains//
void PositionControl::setPlanarPositionGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	//Use values from the users parameters,this depends on number of fans
	_gain_planar_pos_p = P;
	_gain_planar_pos_i = I;
	_gain_planar_pos_d = D;
}
void PositionControl::setPlanarVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	//Use values from the users parameters,this depends on number of fans
	_gain_planar_vel_p = P;
	_gain_planar_vel_i = I;
	_gain_planar_vel_d = D;
}
//Planar Gains End//

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setPlanarThrustLimits(const float min, const float max,const float planar_threshold)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_planar_thr_min = math::max(min, 10e-4f);
	_lim_planar_thr_max = max;
	_planar_threshold = planar_threshold;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt, const int vectoring_att_mode,bool planar_flight)
{
	bool valid = _inputValid();

	if (valid) {

	// if (vectoring_att_mode > 6 || vectoring_att_mode< 0) {
	// 	// PX4_ERR("Vectoring Mode parameter set to unknown value!");
	// }

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;//yaw control can be separated based on 2 matrices
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	//check value for the switch

	bool distance_flag=false;
	float error_xy=sqrt(pow((_pos_sp(0) - _pos(0)),2)+pow((_pos_sp(1) - _pos(1)),2));
	distance_flag= (error_xy>=_planar_threshold)?true:false;
	//Distance becomes nan during manual motion
	bool moving_flag=false;
	moving_flag=!PX4_ISFINITE(error_xy)?true:false;
	//Conditions for planar motion
	//Vectoring mode on or off only
	planar_flag=(planar_flight||distance_flag||moving_flag)?true:false;

	// PX4_INFO("Current position %f %f %f ",double(_pos(0)) ,double(_pos(1)), double(_pos(2)));
	// PX4_INFO("Current setpoint %f %f %f ",double(_pos_sp(0)) ,double(_pos_sp(1)), double(_pos_sp(2)));
	// PX4_INFO("Distancer to error %f %s", double(error_xy),planar_flag?"Planar" :"Tilting");
	// PX4_INFO("MOving flag is  %s",moving_flag?"true" :"false");

	switch (vectoring_att_mode) {


		case 1:
		if (planar_flag){
		_single_positionControl(dt,_yaw_sp);
		_single_velocityControl(dt,_yaw_sp);
		// PX4_INFO("combined planar");
		}
		else {
		_positionControl();
		_velocityControl(dt);
		// PX4_INFO("tilted");
		}
		break;//here

	case 2:

		if (planar_flag){
		_combined_positionControl(dt,_yaw_sp);
		_combined_velocityControl(dt,_yaw_sp);
		// PX4_INFO("combined planar");
		}
		else {
		_positionControl();
		_velocityControl(dt);
		// PX4_INFO("tilted");
		}
		break;
	case 3:
		_positionControl();
		_velocityControl(dt);
		break;//here

	default:
		_positionControl();
		_velocityControl(dt);
		}
	}


	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}


void PositionControl::_positionControl()
{	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
	// PX4_INFO("Position setpoint %f %f %f",(double)_pos_sp(0),(double)_pos_sp(1),(double)_pos_sp(2));
}


void PositionControl::_velocityControl(const float dt)
{

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
	//att_sp.thrust_body[2] = -thr_sp.length();

	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	// Estimate the optimal tilt angle and direction to conteract the wind
	// Prioritize vertical control while keeping a horizontal margin
	//Mode dependant with additional actuators is not needed
	Vector2f thrust_sp_xy(_thr_sp);
	float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / (_hover_thrust*2));
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
	// PX4_INFO("Th %f %f %f",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));
	// PX4_INFO("Vel %f %f %f",(double)_vel_sp(0),(double)_vel_sp(1),(double)_vel_sp(2));

}
void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	// PX4_INFO("Acceleration setpoint %f %f %f",(double)_acc_sp(0),(double)_acc_sp(1),(double)_acc_sp(2));

	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	_thr_sp = body_z * collective_thrust;
}

///////////////// PLANAR CONTROL PID/////////////////

void PositionControl::_planar_positionControl(const float dt, const float yaw_sp)
{
	//position error
	Vector3f pos_error = (_pos_sp - _pos);
	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p) + _pos_int - _vel.emult(_gain_planar_pos_d);
	// Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);
	// vel_sp_position(0)+=_pos_int(0) -_vel(0)*_gain_planar_pos_d(0);
	// vel_sp_position(1)+=_pos_int(1) - _vel(1)*_gain_planar_pos_d(1);

	// Update integral part of velocity control
	//separate based on each individual velocity component
	_pos_int =_pos_int + pos_error.emult(_gain_planar_pos_i) * dt;

	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	// ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Update integral part of velocity control
	//separate based on each individual velocity component
	_pos_int = pos_error.emult(_gain_planar_pos_i) * dt;

	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, yaw_sp}};


	Vector3f vel_sp_xy=_rotation * Vector3f{_vel_sp(0),_vel_sp(1),0};
	vel_sp_xy(0) = math::constrain(vel_sp_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);
	vel_sp_xy(1) = math::constrain(vel_sp_xy(1), -_lim_vel_horizontal, _lim_vel_horizontal);
	vel_sp_xy=_rotation2*Vector3f{vel_sp_xy(0),vel_sp_xy(1),0};
	_vel_sp.xy()=vel_sp_xy.xy();
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}



void PositionControl::_planar_velocityControl(const float dt,const float yaw_sp)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f acc_sp_velocity = vel_error.emult(_gain_planar_vel_p) + _vel_int - _vel_dot.emult(_gain_planar_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_planar_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	const float thrust_max_squared = math::sq(_lim_thr_max);

	//Vertical thrust
	const float thrust_z_max_squared = thrust_max_squared;// - math::sq(allocated_horizontal_thrust);
	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));


	//////Compare the merit of using an anti windup
	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// Integrator anti-windup in vertical direction

	//Rotate the thrust
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, yaw_sp}};

	Vector3f thr_sp_xy=_rotation * Vector3f{_thr_sp(0),_thr_sp(1),0};
	Vector3f vel_xy_error=_rotation * Vector3f{vel_error(0),vel_error(1),0};
	//separate the thrust for each sign

	if(thr_sp_xy(0)>=0.0f)
	{
		if ((thr_sp_xy(0) >= _lim_planar_thr_max && vel_xy_error(0) >= 0.0f) ||
		(thr_sp_xy(0)<= _lim_planar_thr_min && vel_xy_error(0) <= 0.0f)) {
		vel_xy_error(0) = 0.f;
		}
	}

	else {
		if ((thr_sp_xy(0) <= -_lim_planar_thr_max && vel_xy_error(0) <= 0.0f) ||
		(thr_sp_xy(0)>= -_lim_planar_thr_min && vel_xy_error(0) >= 0.0f)) {
		vel_xy_error(0) = 0.f;
		}

	}

	if(thr_sp_xy(1)>=0.0f)
	{
		if ((thr_sp_xy(1) >= _lim_planar_thr_max && vel_xy_error(1) >= 0.0f) ||
		(thr_sp_xy(1)<= _lim_planar_thr_min && vel_xy_error(1) <= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}
	}

	else {
		if ((thr_sp_xy(1) <= -_lim_planar_thr_max && vel_xy_error(1) <= 0.0f) ||
		(thr_sp_xy(1)>= -_lim_planar_thr_min && vel_xy_error(1) >= 0.0f)) {
		vel_xy_error(1) = 0.f;
		}

	}

	thr_sp_xy(0)=thr_sp_xy(0)>=0.0f? math::min(thr_sp_xy(0),_lim_planar_thr_max): math::max(thr_sp_xy(0),-_lim_planar_thr_max);
	thr_sp_xy(1)=thr_sp_xy(1)>=0.0f? math::min(thr_sp_xy(1),_lim_planar_thr_max): math::max(thr_sp_xy(1),-_lim_planar_thr_max);


	thr_sp_xy=_rotation2*Vector3f{thr_sp_xy(0),thr_sp_xy(1),0};
	vel_xy_error=_rotation2*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
	_thr_sp.xy()=thr_sp_xy.xy();
	vel_error.xy()=vel_xy_error.xy();


	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	//separate based on each individual velocity component
	_vel_int += vel_error.emult(_gain_planar_vel_i) * dt;


	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));
	// PX4_INFO("Ve %f %f %f",(double)_vel_sp(0),(double)_vel_sp(1),(double)_vel_sp(2));



}

void PositionControl::_planar_accelerationControl(const float yaw_sp)
{

	//divide by acceleration
	Vector3f body_z = Vector3f(0, 0, CONSTANTS_ONE_G).normalized();
	Vector3f thrz;
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	float x_thrust= _acc_sp(0)*(_hover_thrust);// use a different value perhaps to scale XY since the hover value changes
	float y_thrust= _acc_sp(1)*(_hover_thrust);//similar to the weight of the uav
	//independent of each other, no need to normalize
	Vector3f bodyxy= Vector3f(x_thrust, y_thrust, 0.0);// normalized the xy vector

	thrz= body_z * collective_thrust;

	// // Project thrust to planned body attitude
	_thr_sp(0) = bodyxy(0);
	_thr_sp(1) = bodyxy(1);
	_thr_sp(2) =thrz(2);
	// PX4_INFO("Thrust Components acceleration %f %f %f",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));
	//this thrust only depends of the Z axis
}
///////////////// PLANAR CONTROL PID/////////////////

///////////////// PLANAR PITCH AND ROLL/////////////////

void PositionControl::_combined_positionControl(const float dt,const float yaw_sp)
{
	//could be calculated based on the current angle (tilt_angle)
	//Based on this the system could determine when to tilt and when planar motion is accessible
	//rotation_matrix(tilted-angle) * thrust_direction, check the planar locations -> @rjros
	//position error
	//check Velocity setpoint direction
	//assume gains are for this mode only, although they could be based on the direction
	// of the vel vector

	// P-position controller
	Vector3f pos_error = _pos_sp - _pos;
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence

	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);// + _pos_int - _vel.emult(_gain_planar_pos_d);
	// Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);
	// vel_sp_position(0)+=_pos_int(0) -_vel(0)*_gain_planar_pos_d(0);
	// vel_sp_position(1)+=_pos_int(1) - _vel(1)*_gain_planar_pos_d(1);

	// Update integral part of velocity control
	//separate based on each individual velocity component
	//_pos_int =_pos_int + pos_error.emult(_gain_planar_pos_i) * dt;

	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f vel_body_xy=_rotation * Vector3f{_vel_sp(0),_vel_sp(1),0};

	//Vel in X axis
	vel_body_xy(0) = math::constrain(vel_body_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);

	//Vel X and Y
	vel_body_xy.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);

	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

}


void PositionControl::_combined_velocityControl(const float dt, const float yaw_sp)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f acc_sp_velocity = vel_error.emult(_gain_planar_vel_p) + _vel_int - _vel_dot.emult(_gain_planar_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_combined_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	//Planar and Tilted case
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f th_body=_rotation*_thr_sp;

	//////Compare the merit of using an anti windup
	// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	// Integrator anti-windup in vertical direction

	//Thrust Z check the pitch effects in the thrust
	Vector2f thrust_sp_xy(0,th_body(1));
	float thrust_sp_xy_norm = thrust_sp_xy.norm();
	float thrust_max_squared = math::sq(_lim_thr_max);
	float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	th_body(2) = math::max(th_body(2), -sqrtf(thrust_z_max_squared));
	// Determine how much horizontal thrust is left after prioritizing vertical control

	float thrust_max_xy_squared = thrust_max_squared - math::sq(th_body(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in Y axis (roll)
	if (thrust_sp_xy_norm > thrust_max_xy) {
		th_body(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
	}

	Vector3f vel_xy_error=_rotation * Vector3f{vel_error(0),vel_error(1),0};
	//separate the thrust for each sign
	if ((th_body(0) >= _lim_planar_thr_max && vel_xy_error(0) >= 0.0f) ||
	(th_body(0)<= _lim_planar_thr_min && vel_xy_error(0) <= 0.0f)) {
	vel_xy_error(0) = 0.f;
	}
	th_body(0)=math::min(th_body(0),_lim_planar_thr_max);

	vel_xy_error=_rotation2*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
	Vector3f th_new=_rotation2*th_body;
	_thr_sp.xy()=th_new.xy();
	vel_error.xy()=vel_xy_error.xy();

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));

	// PX4_INFO("Thrust  %f %f %f",(double)th_body(0),(double)th_body(1),(double)th_body(2));
	// PX4_INFO("Thrust Body %f %f %f",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));




}
void PositionControl::_combined_accelerationControl(const float yaw_sp)
{
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f body_accel_sp=_rotation*_acc_sp;
	Vector3f th_body=Vector3f{0.0,0.0,0.0};

	//YZ
	Vector3f body_z = Vector3f(0.0f, -body_accel_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	float collective_thrust = body_accel_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);

	//Thrust back to rotation
	th_body=body_z * collective_thrust;
	th_body(0)=body_accel_sp(0)*_hover_thrust;
	_thr_sp=_rotation2*th_body;

}

///////////////// PLANAR PITCH AND ROLL END /////////////////
///////////////// SINGLE PLANAR PITCH CONTROL PID/////////////////
void PositionControl::_single_positionControl(const float dt,const float yaw_sp)
{

	// P-position controller
	Vector3f pos_error = _pos_sp - _pos;
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence

	Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);// + _pos_int - _vel.emult(_gain_planar_pos_d);
	// Vector3f vel_sp_position = pos_error.emult(_gain_planar_pos_p);
	// vel_sp_position(0)+=_pos_int(0) -_vel(0)*_gain_planar_pos_d(0);
	// vel_sp_position(1)+=_pos_int(1) - _vel(1)*_gain_planar_pos_d(1);

	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f vel_body_xy=_rotation * Vector3f{_vel_sp(0),_vel_sp(1),0};
	if(vel_body_xy(0)>0)
	{
		vel_body_xy(0) = math::constrain(vel_body_xy(0), -_lim_vel_horizontal, _lim_vel_horizontal);
		vel_body_xy(1) = math::constrain(vel_body_xy(1), -_lim_vel_horizontal, _lim_vel_horizontal);

	}
	else
	{
		vel_body_xy.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);

	}
	vel_body_xy.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);

	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

}


void PositionControl::_single_velocityControl(const float dt, const float yaw_sp)
{
	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	//gains are the same as the ones used in the tilting mode, this should be adjusted by the user
	//The parametes should be gain_vel_p and gain_vel_d
	Vector3f acc_sp_velocity = vel_error.emult(_gain_planar_vel_p) + _vel_int - _vel_dot.emult(_gain_planar_vel_d);

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_single_accelerationControl(yaw_sp);
	//Vertical acceleration
	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	(_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
	vel_error(2) = 0.f;
	}

	//Planar and Tilted case
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f th_body=_rotation*_thr_sp;

	if (th_body(0)>0)
	{
		//////Compare the merit of using an anti windup
		// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		// Integrator anti-windup in vertical direction

		//Thrust Z check the pitch effects in the thrust
		Vector2f thrust_sp_xy(0,th_body(1));
		float thrust_sp_xy_norm = thrust_sp_xy.norm();
		float thrust_max_squared = math::sq(_lim_thr_max);
		float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
		float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

		// Saturate maximal vertical thrust
		th_body(2) = math::max(th_body(2), -sqrtf(thrust_z_max_squared));
		// Determine how much horizontal thrust is left after prioritizing vertical control

		float thrust_max_xy_squared = thrust_max_squared - math::sq(th_body(2));
		float thrust_max_xy = 0;

		if (thrust_max_xy_squared > 0) {
			thrust_max_xy = sqrtf(thrust_max_xy_squared);
		}

		// Saturate thrust in Y axis (roll)
		if (thrust_sp_xy_norm > thrust_max_xy) {
			th_body(1) = thrust_sp_xy(1) / thrust_sp_xy_norm * thrust_max_xy;
		}

		Vector3f vel_xy_error=_rotation * Vector3f{vel_error(0),vel_error(1),0};
		//separate the thrust for each sign
		if ((th_body(0) >= _lim_planar_thr_max && vel_xy_error(0) >= 0.0f) ||
		(th_body(0)<= _lim_planar_thr_min && vel_xy_error(0) <= 0.0f)) {
		vel_xy_error(0) = 0.f;
		}
		th_body(0)=math::min(th_body(0),_lim_planar_thr_max);

		vel_xy_error=_rotation2*Vector3f{vel_xy_error(0),vel_xy_error(1),0};
		Vector3f th_new=_rotation2*th_body;
		_thr_sp.xy()=th_new.xy();
		vel_error.xy()=vel_xy_error.xy();

	}

	else
	{

		Vector2f thrust_sp_xy(_thr_sp);
		float thrust_sp_xy_norm = thrust_sp_xy.norm();
		const float thrust_max_squared = math::sq(_lim_thr_max);

		const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);

		const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

		// Saturate maximal vertical thrust
		_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

		// Determine how much horizontal thrust is left after prioritizing vertical control
		const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
		float thrust_max_xy = 0;

		if (thrust_max_xy_squared > 0) {
			thrust_max_xy = sqrtf(thrust_max_xy_squared);
		}

		// Saturate thrust in horizontal direction
		if (thrust_sp_xy_norm > thrust_max_xy) {
			_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
		}

		// // Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
		// // see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / (_hover_thrust*2));
		const float arw_gain = 2.f / _gain_vel_p(0);
		vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	}
		// Make sure integral doesn't get NAN
		ControlMath::setZeroIfNanVector3f(vel_error);
		// Update integral part of velocity control
		_vel_int += vel_error.emult(_gain_vel_i) * dt;

		// limit thrust integral
		_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));

		// PX4_INFO("Thrust  %f %f %f",(double)th_body(0),(double)th_body(1),(double)th_body(2));
		// PX4_INFO("Thrust Body %f %f %f",(double)_thr_sp(0),(double)_thr_sp(1),(double)_thr_sp(2));




}
void PositionControl::_single_accelerationControl(const float yaw_sp)
{
	//Force in the X axis of the body frame must be separated from the acceleration sp.
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -_yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, _yaw_sp}};
	Vector3f body_accel_sp=_rotation*_acc_sp;
	Vector3f th_body=Vector3f{0.0,0.0,0.0};

	if (body_accel_sp(0)>0.0f)
	{
		//YZ
		Vector3f body_z = Vector3f(0.0f, -body_accel_sp(1), CONSTANTS_ONE_G).normalized();
		ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
		float collective_thrust = body_accel_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
		collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
		collective_thrust = math::min(collective_thrust, -_lim_thr_min);

		//Thrust back to rotation
		th_body=body_z * collective_thrust;
		th_body(0)=body_accel_sp(0)*_hover_thrust;
		_thr_sp=_rotation2*th_body;

	}
	else
	{
		// Assume standard acceleration due to gravity in vertical direction for attitude generation
		Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
		// PX4_INFO("Acceleration setpoint %f %f %f",(double)_acc_sp(0),(double)_acc_sp(1),(double)_acc_sp(2));
		ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
		// Scale thrust assuming hover thrust produces standard gravity
		float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
		// Project thrust to planned body attitude
		collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
		collective_thrust = math::min(collective_thrust, -_lim_thr_min);
		_thr_sp = body_z * collective_thrust;

	}


}


///////////////// SINGLE PLANAR PITCH CONTROL PID/////////////////
bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

// void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
// {
// 	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
// 	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
// }

void PositionControl::getAttitudeSetpoint(const matrix::Quatf &att, const int vectoring_att_mode,
					vehicle_attitude_setpoint_s &attitude_setpoint, thrust_vectoring_attitude_status_s &thrust_vectoring_status,
					bool planar_flight)
					const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, att, vectoring_att_mode,attitude_setpoint, thrust_vectoring_status,planar_flag);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
