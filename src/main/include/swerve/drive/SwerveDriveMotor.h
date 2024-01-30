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
  virtual ~SwerveDriveMotor();

  /**
   * Runs a motor with percent ouput/duty cycle
   * @param speed Percent output [-1, 1]
   */
  virtual void Set(double speed) = 0;

  /**
   * Sets the target velocity of the mechanism in RPM
   * @param rpm Target velocity in RPM
   * @param feedfoward Arbitrary feedfoward to add
   */
  virtual void SetVelocity(units::revolutions_per_minute_t rpm, units::volt_t feedforward = 0_V) = 0;

  /**
   * Sets the target velocity of the mechanism required to reach mps meters per second
   * @param mps Target velocity in meters per second
   * @param feedfoward Arbitrary feedfoward to add
   */
  virtual void SetLinearVelocity(units::meters_per_second_t mps, units::volt_t feedforward = 0_V) = 0;

  /**
   * Get the velocity of the mechanism in RPM
   * @return Current velocity in RPM
   */
  virtual units::revolutions_per_minute_t GetVelocity() = 0;

  /**
   * Get the linearized velocity of the mechanism in meters per second
   * @return Current velocity in meters per second
   */
  virtual units::meters_per_second_t GetLinearVelocity() = 0;

  /**
   * Get the position of the mechanism in rotations
   * @return Rotations
   */
  virtual units::turn_t GetPosition() = 0;

  /**
   * Get the displacement from the number of rotations of the wheel
   * @return Meters
   */
  virtual units::meter_t GetDisplacement() = 0;

  /**
   * Configure the motor settings
   */
  virtual void Config() = 0;

  /**
   * Get the underlying motor controller wrapped by this class
   * @return The motor controller
   */
  virtual frc::MotorController* GetMotor() = 0;
};
