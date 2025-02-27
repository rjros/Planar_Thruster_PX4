/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.hpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <ActuatorEffectiveness.hpp>
#include <ActuatorEffectivenessMultirotor.hpp>
#include <ActuatorEffectivenessStandardVTOL.hpp>
#include <ActuatorEffectivenessTiltrotorVTOL.hpp>
#include <ActuatorEffectivenessTailsitterVTOL.hpp>
#include <ActuatorEffectivenessRoverAckermann.hpp>
#include <ActuatorEffectivenessRoverDifferential.hpp>
#include <ActuatorEffectivenessFixedWing.hpp>
#include <ActuatorEffectivenessMCTilt.hpp>
#include <ActuatorEffectivenessCustom.hpp>
#include <ActuatorEffectivenessUUV.hpp>
#include <ActuatorEffectivenessHelicopter.hpp>

//Custom
#include <ActuatorEffectivenessCoaxialThrusters.hpp>

#include <ControlAllocation.hpp>
#include <ControlAllocationPseudoInverse.hpp>
#include <ControlAllocationSequentialDesaturation.hpp>

#include <lib/matrix/matrix/math.hpp>

#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/actuator_servos_trim.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/failure_detector_status.h>
//CUSTOM
#include <uORB/topics/thrust_vectoring_attitude_status.h>

//CUSTOM

class ControlAllocator : public ModuleBase<ControlAllocator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static constexpr int NUM_ACTUATORS = ControlAllocation::NUM_ACTUATORS;
	static constexpr int NUM_AXES = ControlAllocation::NUM_AXES;

	static constexpr int MAX_NUM_MOTORS = actuator_motors_s::NUM_CONTROLS;
	static constexpr int MAX_NUM_SERVOS = actuator_servos_s::NUM_CONTROLS;

	using ActuatorVector = ActuatorEffectiveness::ActuatorVector;

	ControlAllocator();

	virtual ~ControlAllocator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	struct ParamHandles {
		param_t slew_rate_motors[MAX_NUM_MOTORS];
		param_t slew_rate_servos[MAX_NUM_SERVOS];
	};

	struct Params {
		float slew_rate_motors[MAX_NUM_MOTORS];
		float slew_rate_servos[MAX_NUM_SERVOS];
	};

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void update_allocation_method(bool force);
	bool update_effectiveness_source();

	void update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason);

	// *** Function used for helicopters throttle***
	float throttleSpoolupProgress();
	uint64_t _armed_time{0};
	float _spoolup_time{10};
	float _throttle_curve[5];
	// *** END ***

	void check_for_motor_failures();

	void publish_control_allocator_status(int matrix_index);

	void publish_actuator_controls();

	// *** CUSTOM ***
	bool update_CA_manual_mode(); // -> checks if the manual orientation change has been triggered
	// *** END ***

	AllocationMethod _allocation_method_id{AllocationMethod::NONE};
	ControlAllocation *_control_allocation[ActuatorEffectiveness::MAX_NUM_MATRICES] {}; 	///< class for control allocation calculations
	int _num_control_allocation{0};
	hrt_abstime _last_effectiveness_update{0};

	enum class EffectivenessSource {
		NONE = -1,
		MULTIROTOR = 0,
		FIXED_WING = 1,
		STANDARD_VTOL = 2,
		TILTROTOR_VTOL = 3,
		TAILSITTER_VTOL = 4,
		ROVER_ACKERMANN = 5,
		ROVER_DIFFERENTIAL = 6,
		MOTORS_6DOF = 7,
		MULTIROTOR_WITH_TILT = 8,
		CUSTOM = 9,
		HELICOPTER_TAIL_ESC = 10,
		HELICOPTER_TAIL_SERVO = 11,
		COAX_COPTER_THRUSTERS=12
	};

	enum class FailureMode {
		IGNORE = 0,
		REMOVE_FIRST_FAILING_MOTOR = 1,
	};

	// *** CUSTOM ***
	enum class CA_ManMode {
		NONE = -1,
		FORWARD = 0,
		BACKWARD = 1,
		DOWNWARD = 2,
		UPWARD = 3,
		X_AXIS_CCW = 4,
		X_AXIS_CW = 5,
		Z_AXIS_CCW = 6,
		Z_AXIS_CW = 7,
	};
	//*** END ***


	EffectivenessSource _effectiveness_source_id{EffectivenessSource::NONE};
	// *** CUSTOM ***
	CA_ManMode _ca_manual_mode_id{CA_ManMode::NONE};
	CA_ManMode ref{CA_ManMode::NONE};
	int mode_value{10};
	//*** END ***



	ActuatorEffectiveness *_actuator_effectiveness{nullptr}; 	///< class providing actuator effectiveness

	uint8_t _control_allocation_selection_indexes[NUM_ACTUATORS * ActuatorEffectiveness::MAX_NUM_MATRICES] {};
	int _num_actuators[(int)ActuatorType::COUNT] {};

	// Inputs
	uORB::SubscriptionCallbackWorkItem _vehicle_torque_setpoint_sub{this, ORB_ID(vehicle_torque_setpoint)};  /**< vehicle torque setpoint subscription */
	uORB::SubscriptionCallbackWorkItem _vehicle_thrust_setpoint_sub{this, ORB_ID(vehicle_thrust_setpoint)};	 /**< vehicle thrust setpoint subscription */

	uORB::Subscription _vehicle_torque_setpoint1_sub{ORB_ID(vehicle_torque_setpoint), 1};  /**< vehicle torque setpoint subscription (2. instance) */
	uORB::Subscription _vehicle_thrust_setpoint1_sub{ORB_ID(vehicle_thrust_setpoint), 1};	 /**< vehicle thrust setpoint subscription (2. instance) */

	// Outputs
	uORB::PublicationMulti<control_allocator_status_s> _control_allocator_status_pub[2] {ORB_ID(control_allocator_status), ORB_ID(control_allocator_status)};

	uORB::Publication<actuator_motors_s>	_actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s>	_actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<actuator_servos_trim_s>	_actuator_servos_trim_pub{ORB_ID(actuator_servos_trim)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _failure_detector_status_sub{ORB_ID(failure_detector_status)};
	//Test

	uORB::Subscription _thrust_vectoring_status_sub{ORB_ID(thrust_vectoring_attitude_status)};
	int16_t prev_orientation{0};
	//Test

	matrix::Vector3f _torque_sp;
	matrix::Vector3f _thrust_sp;
	bool _publish_controls{true};

	//Custom//
	float tilt_angle{0};
	matrix::Vector<float, NUM_ACTUATORS> servo_sp;

	//END//

	// Reflects motor failures that are currently handled, not motor failures that are reported.
	// For example, the system might report two motor failures, but only the first one is handled by CA
	uint16_t _handled_motor_failure_bitmask{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	bool _armed{false};
	hrt_abstime _last_run{0};
	hrt_abstime _timestamp_sample{0};
	hrt_abstime _last_status_pub{0};

	ParamHandles _param_handles{};
	Params _params{};
	bool _has_slew_rate{false};
	bool new_mode{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CA_AIRFRAME>) _param_ca_airframe,
		(ParamInt<px4::params::CA_METHOD>) _param_ca_method,
		(ParamInt<px4::params::CA_FAILURE_MODE>) _param_ca_failure_mode,
		(ParamInt<px4::params::CA_R_REV>) _param_r_rev,
		// *** CUSTOM ***
		(ParamInt<px4::params::CA_MAN_ANGLE>) _param_ca_man_angle,
		(ParamInt<px4::params::CA_INDEX>) _param_ca_index,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time,

		(ParamFloat<px4::params::CA_COAX_THR_C0>) _param_ca_coax_thr_c0,
		(ParamFloat<px4::params::CA_COAX_THR_C1>) _param_ca_coax_thr_c1,
		(ParamFloat<px4::params::CA_COAX_THR_C2>) _param_ca_coax_thr_c2,
		(ParamFloat<px4::params::CA_COAX_THR_C3>) _param_ca_coax_thr_c3,
		(ParamFloat<px4::params::CA_COAX_THR_C4>) _param_ca_coax_thr_c4,

		//*** END ***
		(ParamInt<px4::params::VECT_ATT_MODE>) _param_vectoring_att_mode


	)

};
