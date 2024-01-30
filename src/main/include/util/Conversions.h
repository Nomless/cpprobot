// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

class Conversions {
 public:
  static units::meters_per_second_t RPSToMPS(units::turns_per_second_t rps, units::meter_t circumference);
  static units::turns_per_second_t MPSToRPS(units::meters_per_second_t velocity, units::meter_t circumference);

  static units::meter_t RotationsToMeters(units::turn_t rotations, units::meter_t circumference);
  static units::turn_t MetersToRotations(units::meter_t meters, units::meter_t circumference);
};
