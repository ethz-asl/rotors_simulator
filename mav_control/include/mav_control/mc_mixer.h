#include <mathlib/mathlib.h>
#include <Eigen/Eigen>

class MultirotorMixer {
public:

	MultirotorMixer();

	struct Rotor {
			float roll_scale;
			float pitch_scale;
			float yaw_scale;

		};

	void mix(const Eigen::Vector4d* desired_moments,Eigen::VectorXd& motor_commands);

private:
	
	const Rotor *_rotors;

	unsigned _rotor_count;

	struct {
		float control[6];
	}inputs;

	struct  {
		float control[6];
	}outputs;


		
};