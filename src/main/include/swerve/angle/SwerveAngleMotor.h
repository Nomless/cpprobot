// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <frc/motorcontrol/MotorController.h>

class SwerveAngleMotor {
 public:
  virtual ~SwerveAngleMotor();

  /**
   * Runs a motor with percent ouput/duty cycle
   * @param speed Percent ouput [-1, 1]
   */
  virtual void Set(double speed) = 0;

  /**
   * Sets the target position of the mechanism in rotations
   * @param rotations Target position in rotations
   */
  virtual void SetPosition(units::turn_t rotations) = 0;

  /**
   * Sets the position that the sensor reads
   * @param rotations Position in rotations
   */
  virtual void SetSensorPosition(units::turn_t rotations) = 0;

  /**
   * Get the position of the motor in rotations
   * @return Position in rotations
   */
  virtual units::turn_t GetPosition() = 0;

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
