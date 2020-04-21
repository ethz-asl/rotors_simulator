
#include "rotors_control/height_controller_hover.h"

#include <cmath>
#include <exception>
#include <iostream>

#define G_CONST (double)9.80665

/*!
 * @brief Constructor for HeightControllerPID class
 *
 */
HeightControllerHover::HeightControllerHover(){}


/*!
 * @brief Update plant variables
 */
void HeightControllerHover::updatePlant(double roll, double pitch, double height)
{
  this->roll = roll;
  this->pitch = pitch;
  this->height = height;
}

/*!
 * @brief Generates thrust command
 */
void HeightControllerHover::generate_thrust_command(double linear_controller_output)
{
  double thrust_nominal = this->drone_mass * G_CONST;
  double req_thrust_body = (1 / (cos(this->roll) * cos(this->pitch))) * (thrust_nominal + linear_controller_output);
  this->thrust_command = req_thrust_body;
}

double HeightControllerHover::getThrust_command() const
{
  return this->thrust_command;
}