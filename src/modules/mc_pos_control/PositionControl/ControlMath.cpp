/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>

#include <float.h>
#include <mathlib/mathlib.h>

using namespace matrix;

namespace ControlMath
{
// void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
// {
// 	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
// 	att_sp.thrust_body[2] = -thr_sp.length();
// }

void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att, const int vectoring_att_mode,
		      vehicle_attitude_setpoint_s &att_sp, thrust_vectoring_attitude_status_s &thrust_vectoring_status,bool planar_flight)
{
	switch (vectoring_att_mode) {

	case 1:
		if (planar_flight){
		thrustToSinglePlanarAttitude(thr_sp, yaw_sp, att,att_sp);
		}
		else {
		bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
		att_sp.thrust_body[2] = -thr_sp.length();
		}
		break;
	case 2:
		if (planar_flight){
		thrustToFixedPitchAttitude(thr_sp, yaw_sp, att,att_sp);
		}
		else {
		bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
		att_sp.thrust_body[2] = -thr_sp.length();
		}
		break;

	case 3:
		bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
		att_sp.thrust_body[2] = -thr_sp.length();
		break;


	default: //Altitude is calculated from the desired thrust direction
		bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
		att_sp.thrust_body[2] = -thr_sp.length();
	}


	// Estimate the optimal tilt angle and direction to counteract the wind
	// Calculate the setpoint z axis
	Vector3f cmd_z;
	matrix::Dcmf R_cmd = matrix::Quatf(att_sp.q_d);

	for (int i = 0; i < 3; i++) {
		cmd_z(i) = R_cmd(i, 2);
	}


	// Calculate the current z axis
	Vector3f curr_z;
	matrix::Dcmf R_body = att;

	for (int i = 0; i < 3; i++) {
		curr_z(i) = R_body(i, 2);
	}


}



////////////////////////////////////////////////////////////////////////////////////////////////////
void thrustToSinglePlanarAttitude(const Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			      vehicle_attitude_setpoint_s &att_sp)
{
	//refers to the forward tilt, and used in the MC and Thrust Vectoring model
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//TEST Fixed Pitch Angle//
	// zero vector, no direction, set safe level value
	//The angles in the rotation could be use select the different modes
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, yaw_sp}};
	Vector3f body_x={0.0f,0.0f,0.0f};
	//check the magnitude of the horizontal vector in the body frame
	Vector3f thrust_sp_xy=_rotation * Vector3f{thr_sp(0),thr_sp(1),thr_sp(2)};
	thrust_sp_xy=_rotation2*Vector3f{thrust_sp_xy(0),0.f,0.f};
	//Increase the x axis by a X factor
	// thrust_sp_xy.normalize();

	Vector3f body_z=-thr_sp;// thrust vector that comes
	// Vector3f body_z=-thrust_sp_xy;// thrust vector modified

	//check thrust vector
	Vector3f thrust_rotated = _rotation * matrix::Vector3f{(float)-body_z(0), (float)-body_z(1), (float)-body_z(2)};
	//
	// ("Thrust in body frame %f %f %f",(double)thrust_rotated(0),(double)thrust_rotated(1),(double)thrust_rotated(2));

	if (thrust_rotated(0)>-0.001f) {
	body_z=_rotation2*matrix::Vector3f{0.f, (float)-thrust_rotated(1),(float)-thrust_rotated(2)};
	body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0f);
	body_z.normalize();

	}
	else
	{
		body_z.normalize();
		if (body_z.norm_squared() < FLT_EPSILON) {
			body_z(2) = 1.f;
		}
		const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};
		// desired body_x axis, orthogonal to body_z
		body_x = y_C % body_z;

	}

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.0f) {
		body_x = -body_x;
	}

	// // vector of desired yaw direction in XY plane, rotated by PI/2
	// Vector3f body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0f);
	body_x.normalize();

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.f;
	}

	// // desired body_y axis
	Vector3f body_y = body_z % body_x;
	// //front case

	Dcmf R_sp;


	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();

	//thrust from the x axis
	if (thrust_rotated(0)>-0.001f) {
	att_sp.thrust_body[0] = thrust_sp_xy.dot(body_x);//value of the thrust
	att_sp.thrust_body[1] = thrust_sp_xy.dot(body_y);// not the same
	att_sp.thrust_body[2] = thr_sp.dot(body_z);//value of the z thrust
	}
	else
	{
	att_sp.thrust_body[0] = 0.0f;
	att_sp.thrust_body[1] = 0.0f;
	att_sp.thrust_body[2] = thr_sp.dot(body_z);//value of the z thrust


	}
	// PX4_INFO("Thrust  %f %f %f",(double)att_sp.thrust_body[0],(double)att_sp.thrust_body[1],(double)att_sp.thrust_body[2]);
	// PX4_INFO("Orientation  %f %f %f",(double)math::degrees(att_sp.roll_body),(double)math::degrees(att_sp.pitch_body),(double)math::degrees(att_sp.yaw_body));

}
////////////////////////////////////////////////////////////////////////////////////////////////////
void thrustToFixedPitchAttitude(const Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			      vehicle_attitude_setpoint_s &att_sp)
{
	//refers to the forward tilt, and used in the MC and Thrust Vectoring model
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//TEST Fixed Pitch Angle//
	// zero vector, no direction, set safe level value
	//The angles in the rotation could be use select the different modes
	matrix::Dcmf _rotation,_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -yaw_sp}};
	_rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, yaw_sp}};
	Vector3f body_x={0.0f,0.0f,0.0f};
	//check the magnitude of the horizontal vector in the body frame
	Vector3f thrust_sp_xy=_rotation * Vector3f{thr_sp(0),thr_sp(1),thr_sp(2)};
	thrust_sp_xy(0)=thrust_sp_xy(0)*1.10f;
	//PX4_INFO("Thrust in the setpoint %f %f %f ",(double)thr_sp(0),(double)thr_sp(1),(double)thr_sp(2));
	thrust_sp_xy=_rotation2*Vector3f{thrust_sp_xy(0),thrust_sp_xy(1),thrust_sp_xy(2)};
	//Increase the x axis by a X factor
	// thrust_sp_xy.normalize();
	//PX4_INFO("Thrust by factor %f %f %f ",(double)thrust_sp_xy(0),(double)thrust_sp_xy(1),(double)thrust_sp_xy(2));
	//PX4_INFO("Thrust in body frame %f %f %f",(double)thrust_rotated(0),(double)thrust_rotated(1),(double)thrust_rotated(2));

	// Vector3f body_z=-thr_sp;// thrust vector that comes
	Vector3f body_z=-thrust_sp_xy;// thrust vector modified


	//check thrust vector
	Vector3f thrust_rotated = _rotation * matrix::Vector3f{(float)-body_z(0), (float)-body_z(1), (float)-body_z(2)};
	// PX4_INFO("Thrust in body frame %f %f %f",(double)thrust_rotated(0),(double)thrust_rotated(1),(double)thrust_rotated(2));

	body_z=_rotation2*matrix::Vector3f{0.f, (float)-thrust_rotated(1),(float)-thrust_rotated(2)};
	body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0f);
	body_z.normalize();

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.0f) {
		body_x = -body_x;
	}

	// // vector of desired yaw direction in XY plane, rotated by PI/2
	// Vector3f body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0f);
	body_x.normalize();

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.f;
	}

	// // desired body_y axis
	Vector3f body_y = body_z % body_x;
	// //front case

	Dcmf R_sp;


	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();


	att_sp.thrust_body[0] = thrust_sp_xy.dot(body_x);
	att_sp.thrust_body[1] = thrust_sp_xy.dot(body_y);
	att_sp.thrust_body[2] = thrust_sp_xy.dot(body_z);

	// PX4_INFO("New Thrust Components %f %f %f",(double)att_sp.thrust_body[0],(double)att_sp.thrust_body[1],(double)att_sp.thrust_body[2]);


}

void thrustToZeroTiltAttitude(const Vector3f &thr_sp, const float yaw_sp, const matrix::Quatf &att,
			      vehicle_attitude_setpoint_s &att_sp)
	{

	//Ignores the commannd and relies on the acceleration or addition based
	// set Z axis to upward direction
	Vector3f body_z = Vector3f(0.f, 0.f, 1.f);

	// desired body_x and body_y axis
	Vector3f body_x = Vector3f(cos(yaw_sp), sin(yaw_sp), 0.0f);
	Vector3f body_y = Vector3f(-sinf(yaw_sp), cosf(yaw_sp), 0.0f);

	Dcmf R_sp;


	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	Quatf q_sp = R_sp;
	q_sp.copyTo(att_sp.q_d);
	att_sp.q_d_valid = true;



	// set the euler angles, for logging only, must not be used for control
	att_sp.roll_body = 0;
	att_sp.pitch_body = 0;
	att_sp.yaw_body = yaw_sp;


	att_sp.thrust_body[0] = thr_sp.dot(body_x);// values tend to be very small
	att_sp.thrust_body[1] = thr_sp.dot(body_y);
	att_sp.thrust_body[2] = thr_sp.dot(body_z);

	// PX4_INFO("New Thrust Components %f %f %f",(double)att_sp.thrust_body[0],(double)att_sp.thrust_body[1],(double)att_sp.thrust_body[2]);


	// PX4_INFO("attitude x, y ,z %f %f %f",(double)att_sp.roll_body,(double)att_sp.pitch_body,(double)att_sp.yaw_body);
	}

//void planar motion (direction)

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{

// zero vector, no direction, set safe level value
	// PX4_INFO("Tilting Body_z %f %f %f",(double)-body_z(0),(double)-body_z(1),(double)-body_z(2));
	matrix::Dcmf _rotation; //_rotation2;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, -yaw_sp}};//rotation to the body frame
	// _rotation2 = matrix::Dcmf{matrix::Eulerf{0.f, 0.f, yaw_sp}};
	//check thrust vector
	// Vector3f thrust_rotated = _rotation * matrix::Vector3f{(float)-body_z(0), (float)-body_z(1), 0};
	// PX4_INFO("Thrust in body z  %f %f %f",(double)body_z(0),(double)body_z(1),(double)body_z(2));
	// PX4_INFO("Thrust in body frame 0 deg %f %f %f",(double)thrust_rotated(0),(double)thrust_rotated(1),(double)thrust_rotated(2));

	// Vector3f thrust_fixed=_rotation2*matrix::Vector3f{0.0f, (float)thrust_rotated(1),(float)thrust_rotated(2)};
	// thrust_fixed.normalize();

	// PX4_INFO("Thrust TILTED BACK %f %f %f",(double)thrust_fixed(0),(double)thrust_fixed(1),(double)thrust_fixed(2));


	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();


	// PX4_INFO("Thrust in body frame %f %f %f",(double)body_z(0),(double)body_z(1),(double)body_z(2));


	// vector of desired yaw direction in XY plane, rotated by PI/2
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// desired body_x axis, orthogonal to body_z
	Vector3f body_x = y_C % body_z;

	// keep nose to front while inverted upside down
	if (body_z(2) < 0.0f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		// desired thrust is in XY plane, set X downside to construct correct matrix,
		// but yaw component will not be used actually
		body_x.zero();
		body_x(2) = 1.0f;
	}

	body_x.normalize();


	// desired body_y axis
	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	// fill rotation matrix
	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);
	att_sp.q_d_valid = true;

	// calculate euler angles, for logging only, must not be used for control
	const Eulerf euler{R_sp};
	att_sp.roll_body = euler.phi();
	att_sp.pitch_body = euler.theta();
	att_sp.yaw_body = euler.psi();
	// PX4_INFO("Thrust %f %f %f",(double)thr,(double)att_sp.thrust_body[1],(double)att_sp.thrust_body[2]);


}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.f) {
			// target waypoint is already behind us
			res = line_b;

		} else {
			// target is in front of us
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		res = d; // go directly to line

		// previous waypoint is still in front of us
		if ((sphere_c - line_a) * ab_norm < 0.f) {
			res = line_a;
		}

		// target waypoint is already behind us
		if ((sphere_c - line_b) * ab_norm > 0.f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

} // ControlMath
