// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <util/SwerveModuleConstants.h>
#include "rev/CANSparkMax.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "rev/SparkRelativeEncoder.h"
#include <subsystems/SwerveModule.h>

class SparkMaxSwerveModule : public SwerveModule {
 public:
  SparkMaxSwerveModule(int module_number, SwerveModuleConstants module_constants);

  void SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop);
  // void SetSpeed(units::meters_per_second_t velcity, bool is_open_loop);
  void SetAngle(frc::SwerveModuleState desired_state);
  // void SetAngle(units::degrees angle);

  rev::SparkPIDController GetAngleController();
  rev::SparkPIDController GetDriveController();
  rev::CANSparkMax GetAngleMotor();
  rev::CANSparkMax GetDriveMotor();
  
  frc::Rotation2d GetAngle();
  units::meters_per_second_t GetVelocity();

  frc::SwerveModulePosition GetPosition();

  void ResetToAbsolute();
 private:
  rev::CANSparkMax angle_motor;
  rev::CANSparkMax drive_motor;

  rev::SparkRelativeEncoder angle_relative_encoder;
  rev::SparkRelativeEncoder drive_relative_encoder;

  rev::SparkPIDController angle_controller;
  rev::SparkPIDController drive_controller;

  void ConfigAngleMotor();
  void ConfigDriveMotor();
};
