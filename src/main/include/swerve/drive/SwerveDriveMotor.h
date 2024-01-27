// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/motorcontrol/MotorController.h>

class SwerveDriveMotor {
 public:
  SwerveDriveMotor();
  
  void Set(double speed);

  void SetVelocity(units::revolutions_per_minute_t rpm);

  void SetLinearVelocity(units::meters_per_second_t mps, double feedforward = 0);

  units::revolutions_per_minute_t GetVelocity();

  units::meters_per_second_t GetLinearVelocity();

  units::turn_t GetPosition();

  units::meter_t GetDisplacement();

  void Config();

  frc::MotorController GetMotor();
};
