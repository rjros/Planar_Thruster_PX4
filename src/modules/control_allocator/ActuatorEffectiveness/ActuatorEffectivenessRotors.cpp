/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessRotors.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ActuatorEffectivenessRotors.hpp"

#include "ActuatorEffectivenessTilts.hpp"

using namespace matrix;

ActuatorEffectivenessRotors::ActuatorEffectivenessRotors(ModuleParams *parent, AxisConfiguration axis_config,
		bool tilt_support)
	: ModuleParams(parent), _axis_config(axis_config), _tilt_support(tilt_support)
{
	for (int i = 0; i < NUM_ROTORS_MAX; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PX", i);
		_param_handles[i].position_x = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PY", i);
		_param_handles[i].position_y = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_PZ", i);
		_param_handles[i].position_z = param_find(buffer);

		if (_axis_config == AxisConfiguration::Configurable) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AX", i);
			_param_handles[i].axis_x = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AY", i);
			_param_handles[i].axis_y = param_find(buffer);
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_AZ", i);
			_param_handles[i].axis_z = param_find(buffer);
		}

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_KM", i);
		_param_handles[i].moment_ratio = param_find(buffer);

		if (_tilt_support) {
			snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_TILT", i);
			_param_handles[i].tilt_index = param_find(buffer);
			PX4_INFO("tilt index %d",_param_handles[i].tilt_index);
		}
	}

	_count_handle = param_find("CA_ROTOR_COUNT");

	updateParams();
}

void ActuatorEffectivenessRotors::updateParams()
{
	ModuleParams::updateParams();

	// PX4_INFO("update parameters in effectiveness");


	int32_t count = 0;
	//int32_t tilting_index=0;

	//int tilting_index=0;


	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	//checkAxis(tilting_index);

	_geometry.num_rotors = math::min(NUM_ROTORS_MAX, (int)count);

	// PX4_INFO("Changed Axis");
	_thrust_vectoring_status_sub.copy(&thrust_vec_status);



	for (int i = 0; i < _geometry.num_rotors; ++i) {
		Vector3f &position = _geometry.rotors[i].position;
		param_get(_param_handles[i].position_x, &position(0));
		param_get(_param_handles[i].position_y, &position(1));
		param_get(_param_handles[i].position_z, &position(2));

		Vector3f &axis = _geometry.rotors[i].axis;
	//Switch which changes based on the mode, and axis_x*modifier
		switch (_axis_config) {
		case AxisConfiguration::Configurable:// could this value be multiplied by something
			param_get(_param_handles[i].axis_x, &axis(0));
			param_get(_param_handles[i].axis_y, &axis(1));
			param_get(_param_handles[i].axis_z, &axis(2));
			break;

		case AxisConfiguration::FixedForward:
			axis = Vector3f(1.f, 0.f, 0.f);
			break;

		case AxisConfiguration::FixedUpwards:
			axis = Vector3f(0.f, 0.f, -1.f);
			break;
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].moment_ratio, &_geometry.rotors[i].moment_ratio);

		if (_tilt_support) {
			int32_t tilt_param{0};
			param_get(_param_handles[i].tilt_index, &tilt_param);
			_geometry.rotors[i].tilt_index = tilt_param - 1;

		} else {
			_geometry.rotors[i].tilt_index = -1;
		}
	}
}

bool
ActuatorEffectivenessRotors::addActuators(Configuration &configuration,bool tiltable)
{
	if (configuration.num_actuators[(int)ActuatorType::SERVOS] > 0) {
		PX4_ERR("Wrong actuator ordering: servos need to be after motors");
		return false;
	}


	_thrust_vectoring_status_sub.copy(&thrust_vec_status);
	float tilt_angle=0;
	int att_tilt_mode=0;

	//Check the current attitude mode of the system
	if (thrust_vec_status.att_mode<2)
		{
			att_tilt_mode=1;
		}
	//check the current value of the man orientation
	switch (thrust_vec_status.manual_orientation) {
			case 0:
				tilt_angle=0;//
				break;
			case 1:
				tilt_angle=thrust_vec_status.forward_angle; //
				break;

			case 2:
				tilt_angle=thrust_vec_status.backward_angle; //
				break;
			case 3:
				tilt_angle=thrust_vec_status.backward_angle; //radians

				break;
			default:
				tilt_angle=0;//radians
	}

	//Fixed Propellers
	int num_actuators=0;
	if(configuration.selected_matrix==0){
		//vertical forces
		num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix],tiltable,tilt_angle,att_tilt_mode);

	// PX4_INFO("Vertical Forces \n");
	// PX4_INFO("num_actuators: %d  \n", num_actuators);

	}
	else if (configuration.selected_matrix == 1)
	{
		//Tilting forces
		num_actuators = computeEffectivenessMatrix(_geometry,
		configuration.effectiveness_matrices[configuration.selected_matrix],
		configuration.num_actuators_matrix[configuration.selected_matrix],tiltable,tilt_angle);
	// PX4_INFO("Tilting Forces \n");
	// PX4_INFO("num_actuators: %d \n matrix ", num_actuators);
	}

	configuration.actuatorsAdded(ActuatorType::MOTORS, num_actuators);

	return true;
}

int
ActuatorEffectivenessRotors::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index,bool tiltable_matrix,float tilt_angle,int att_mode)
{

	int num_actuators = 0;
	matrix::Dcmf _rotation;
	_rotation = matrix::Dcmf{matrix::Eulerf{0.f, tilt_angle,0.f}};

	//Used to check change in the following code

	// PX4_INFO("Total number of rotors %d ",geometry.num_rotors);
	if(tiltable_matrix) {
	// PX4_INFO("Fixed matrix");
	//Use only for tilted propellers
		for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
		break;
		}
		// PX4_INFO("Actuator_start index %d", i);

		++num_actuators;
		//original axis
		Vector3f axis = geometry.rotors[i].axis;

		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f position = geometry.rotors[i].position;

		// Get coefficients

		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;
		//index for the fixed values
		if(i>=6)

		{	ct=geometry.rotors[i].thrust_coef*att_mode;
			axis=_rotation*axis*att_mode;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;
		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
			}
		}
	}

	else {
		for (int i = 0; i < math::min(NUM_ROTORS_MAX, geometry.num_rotors); i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f axis = geometry.rotors[i].axis;

		// Normalize axis
		float axis_norm = axis.norm();

		if (axis_norm > FLT_EPSILON) {
			axis /= axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float km = geometry.rotors[i].moment_ratio;

		if (geometry.propeller_torque_disabled) {
			km = 0.f;
		}

		if (geometry.propeller_torque_disabled_non_upwards) {
			bool upwards = fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f;

			if (!upwards) {
				km = 0.f;
			}
		}

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		matrix::Vector3f thrust = ct * axis;

		// Compute moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(axis) - ct * km * axis;

		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
		}

	}


	return num_actuators;
}

uint32_t ActuatorEffectivenessRotors::updateAxisFromTilts(const ActuatorEffectivenessTilts &tilts,
		float collective_tilt_control)
{
	if (!PX4_ISFINITE(collective_tilt_control)) {
		collective_tilt_control = -1.f;
	}

	uint32_t nontilted_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		int tilt_index = _geometry.rotors[i].tilt_index;

		if (tilt_index == -1 || tilt_index >= tilts.count()) {
			nontilted_motors |= 1u << i;
			continue;
		}

		const ActuatorEffectivenessTilts::Params &tilt = tilts.config(tilt_index);
		const float tilt_angle = math::lerp(tilt.min_angle, tilt.max_angle, (collective_tilt_control + 1.f) / 2.f);
		const float tilt_direction = math::radians((float)tilt.tilt_direction);
		_geometry.rotors[i].axis = tiltedAxis(tilt_angle, tilt_direction);
	}

	return nontilted_motors;
}

Vector3f ActuatorEffectivenessRotors::tiltedAxis(float tilt_angle, float tilt_direction)
{
	Vector3f axis{0.f, 0.f, -1.f};
	return Dcmf{Eulerf{0.f, -tilt_angle, tilt_direction}} * axis;
}

uint32_t ActuatorEffectivenessRotors::getMotors() const
{
	uint32_t motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		motors |= 1u << i;
	}

	return motors;
}

uint32_t ActuatorEffectivenessRotors::getUpwardsMotors() const
{
	uint32_t upwards_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (fabsf(axis(0)) < 0.1f && fabsf(axis(1)) < 0.1f && axis(2) < -0.5f) {
			upwards_motors |= 1u << i;
		}
	}

	return upwards_motors;
}

uint32_t ActuatorEffectivenessRotors::getForwardsMotors() const
{
	uint32_t forward_motors = 0;

	for (int i = 0; i < _geometry.num_rotors; ++i) {
		const Vector3f &axis = _geometry.rotors[i].axis;

		if (axis(0) > 0.5f && fabsf(axis(1)) < 0.1f && fabsf(axis(2)) < 0.1f) {
			forward_motors |= 1u << i;
		}
	}

	return forward_motors;
}

bool
ActuatorEffectivenessRotors::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	return addActuators(configuration);
}

// void
// ActuatorEffectivenessRotors::checkAxis(tilting_index){

// }
