// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <frc/motorcontrol/MotorController.h>

class SwerveAngleMotor {
 public:
  SwerveAngleMotor();

  /**
   * Runs a motor with percent ouput/duty cycle
   * @param speed Percent ouput [-1, 1]
   */
  void Set(double speed);

  /**
   * Sets the target position of the mechanism in rotations
   * @param rotations Target position in rotations
   */
  void SetPosition(units::turn_t rotations);

  /**
   * Sets the position that the sensor reads
   * @param rotations Position in rotations
   */
  void SetSensorPosition(units::turn_t rotations);

  /**
   * Get the position of the motor in rotations
   * @return Position in rotations
   */
  units::turn_t GetPosition();

  /**
   * Configure the motor settings
   */
  void Config();

  /**
   * Get the underlying motor controller wrapped by this class
   * @return The motor controller
   */
  frc::MotorController* GetMotor();
};
