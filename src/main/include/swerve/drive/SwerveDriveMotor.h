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
  virtual void Set(double speed) = 0;

  virtual void SetVelocity(units::revolutions_per_minute_t rpm) = 0;

  virtual void SetLinearVelocity(units::meters_per_second_t mps, double feedforward = 0) = 0;

  virtual units::revolutions_per_minute_t GetVelocity() = 0;

  virtual units::meters_per_second_t GetLinearVelocity() = 0;

  virtual units::turn_t GetPosition() = 0;

  virtual units::meter_t GetDisplacement() = 0;

  virtual void Config() = 0;

  virtual frc::MotorController* GetMotor() = 0;
};
