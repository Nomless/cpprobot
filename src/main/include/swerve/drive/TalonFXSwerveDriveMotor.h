// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "swerve/drive/SwerveDriveMotor.h"
#include <ctre/phoenix6/TalonFX.hpp>

class TalonFXSwerveDriveMotor : public SwerveDriveMotor {
 private:
  ctre::phoenix6::hardware::TalonFX motor;
  ctre::phoenix6::controls::VelocityVoltage control;
 public:
  TalonFXSwerveDriveMotor(int can_id);
  ~TalonFXSwerveDriveMotor();

  /**
   * Runs a motor with percent ouput/duty cycle
   * @param speed Percent output [-1, 1]
   */
  void Set(double speed) override;

  /**
   * Sets the target velocity of the mechanism in RPM
   * @param rpm Target velocity in RPM
   * @param feedfoward Arbitrary feedfoward to add
   */
  void SetVelocity(units::revolutions_per_minute_t rpm, units::volt_t feedfoward = 0_V) override;

  /**
   * Sets the target velocity of the mechanism required to reach mps meters per second
   * @param mps Target velocity in meters per second
   * @param feedfoward Arbitrary feedfoward to add
   */
  void SetLinearVelocity(units::meters_per_second_t mps, units::volt_t feedforward = 0_V) override;

  /**
   * Get the velocity of the mechanism in RPM
   * @return Current velocity in RPM
   */
  units::revolutions_per_minute_t GetVelocity() override;

  /**
   * Get the linearized velocity of the mechanism in meters per second
   * @return Current velocity in meters per second
   */
  units::meters_per_second_t GetLinearVelocity() override;

  /**
   * Get the position of the mechanism in rotations
   * @return Rotations
   */
  units::turn_t GetPosition() override;

  /**
   * Get the displacement from the number of rotations of the wheel
   * @return Meters
   */
  units::meter_t GetDisplacement() override;

  /**
   * Configure the motor settings
   */
  void Config() override;

  /**
   * Get the underlying motor controller wrapped by this class
   * @return The motor controller
   */
  ctre::phoenix6::hardware::TalonFX* GetMotor() override;
};
