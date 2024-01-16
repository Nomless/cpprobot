// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

class CanCoderConversions {
 public:
  static units::degree_t ToDegrees(units::turn_t can_coder, double gear_ratio = 1);
  static units::turn_t ToCanCoder(units::degree_t degrees, double gear_ratio = 1);

};

class TalonFXConversions {
 public:
  static units::degree_t ToDegrees(units::turn_t rotations, double gear_ratio = 1);
  static units::turn_t ToFalcon(units::degree_t degrees, double gear_ratio = 1);

  static units::revolutions_per_minute_t ToRpm(units::turns_per_second_t rps, double gear_ratio = 1);
  static units::turns_per_second_t ToFalcon(units::revolutions_per_minute_t rpm, double gear_ratio = 1);

  static units::meters_per_second_t ToMetersPerSecond(units::turns_per_second_t rps, units::meter_t circumference, double gear_ratio = 1);
  static units::turns_per_second_t ToFalcon(units::meters_per_second_t velocity, units::meter_t circumference, double gear_ratio = 1);

  static units::meter_t ToMeters(units::turn_t rotations, units::meter_t circumference, double gear_ratio = 1);
  static units::turn_t ToFalcon(units::meter_t meters, units::meter_t circumference, double gear_ratio = 1);
};

namespace SparkMaxConversions {

}
