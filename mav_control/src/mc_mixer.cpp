/*
 * mc_mixer.cpp
 *
 *  Created on: Sep 17, 2014
 *      Author: roman
 */

#include <mav_control/mc_mixer.h>
#include <mathlib/mathlib.h>


 // const MultirotorMixer::Rotor _config_quadshot[] =
 // {
 // { -0.3223,  0.9466,   0.4242 },
 // {  0.3223, -0.9466,   1.0000 },
 // {  0.3223,  0.9466,  -0.4242 },
 // { -0.3223, -0.9466,  -1.0000 },
 // };


const MultirotorMixer::Rotor _config_quadshot[] = {
	{  0.000000,  -1.000000, -1.00 },
	{ -0.000000, 1.000000, -1.00 },
	{  1.000000,  0.000000,  1.00 },
	{  -1.000000,  0.000000,  1.00 },
};

	const MultirotorMixer::Rotor *_config_index = { &_config_quadshot[0]

	};

MultirotorMixer::MultirotorMixer() :
		_rotor_count(4), _rotors(_config_index) {

	memset(&inputs,0,sizeof(inputs));
	memset(&outputs,0,sizeof(outputs));

} 



void MultirotorMixer::mix(const Eigen::Vector4d* desired_moments,Eigen::VectorXd& motor_commands) {
	for (unsigned i = 0; i < 4; i++)
	{
		this->inputs.control[i] = (*desired_moments)(i);
	}


	float roll = math::constrain(inputs.control[0], -1.0f, 1.0f);
	float pitch = math::constrain(inputs.control[1], -1.0f, 1.0f);
	float yaw = math::constrain(inputs.control[2], -1.0f, 1.0f);
	float thrust = math::constrain(inputs.control[3], 0.0f, 1.0f);
	float min_out = 0.0f;
	float max_out = 0.0f;

	/* perform initial mix pass yielding unbounded outputs, ignore yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * _rotors[i].roll_scale
				+ pitch * _rotors[i].pitch_scale + thrust;

		/* limit yaw if it causes outputs clipping */
		if (out >= 0.0f && out < -yaw * _rotors[i].yaw_scale) {
			yaw = -out / _rotors[i].yaw_scale;
		}

		/* calculate min and max output values */
		if (out < min_out) {
			min_out = out;
		}
		if (out > max_out) {
			max_out = out;
		}

		outputs.control[i] = out;
	}
	/* scale down roll/pitch controls if some outputs are negative, don't add yaw, keep total thrust */
	if (min_out < 0.0f) {
		float scale_in = thrust / (thrust - min_out);

		/* mix again with adjusted controls */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] = scale_in
					* (roll * _rotors[i].roll_scale
							+ pitch * _rotors[i].pitch_scale) + thrust;
		}

	} else {
		/* roll/pitch mixed without limiting, add yaw control */
		for (unsigned i = 0; i < _rotor_count; i++) {
			outputs.control[i] += yaw * _rotors[i].yaw_scale;
		}
	}

	/* scale down all outputs if some outputs are too large, reduce total thrust */
	float scale_out;
	if (max_out > 1.0f) {
		scale_out = 1.0f / max_out;

	} else {
		scale_out = 1.0f;
	}

	/* scale outputs to range _idle_speed..1, and do final limiting */
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs.control[i] = math::constrain(outputs.control[i], 0.0f, 1.0f);
		motor_commands(i) = outputs.control[i];
	}

	
	


}

