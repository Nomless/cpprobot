// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <swerve/angle/SwerveAngleMotor.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <units/angle.h>

class TalonFXSwerveAngleMotor : public SwerveAngleMotor {
 private:
  ctre::phoenix6::hardware::TalonFX motor;
  ctre::phoenix6::controls::PositionVoltage control;

 public:
  TalonFXSwerveAngleMotor(int can_id);
  ~TalonFXSwerveAngleMotor();

  /**
   * Runs a motor with percent ouput/duty cycle
   * @param speed Percent ouput [-1, 1]
   */
  inline void Set(double speed) override;

  /**
   * Sets the target position of the mechanism in rotations
   * @param rotations Target position in rotations
   */
  inline void SetPosition(units::turn_t rotations) override;

  /**
   * Sets the position that the sensor reads
   * @param rotations Position in rotations
   */
  void SetSensorPosition(units::turn_t rotations) override;

  /**
   * Get the position of the motor in rotations
   * @return Position in rotations
   */
  inline units::turn_t GetPosition() override;

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
