

#ifndef HEIGHT_CONTROLLER_HOVER_HPP
#define HEIGHT_CONTROLLER_HOVER_HPP

// Altitude controller consisting of feedback linearization and a linear controller about hover
// Takes ref height and outputs thrust command
class HeightControllerHover
{
private:


  // Height Setpoint
  double height_ref = 0.0;

  // Plant variables
  double roll = 0.0;
  double pitch = 0.0;
  double height = 0.0;

  // Controller output
  double thrust_command = 0.0;

public:
  // Constants
  double drone_mass = 0.5;

  // Constructor
   HeightControllerHover();
//  ~HeightControllerHover();
  // Plant state updates
  void updatePlant(double roll, double pitch, double height);
  // Controller update
  void generate_thrust_command(double linear_controller_output);
  // Accessor functions (for debugging)
  double getThrust_command() const;
};

#endif  // HEIGHT_CONTROLLER_HOVER_HPP
