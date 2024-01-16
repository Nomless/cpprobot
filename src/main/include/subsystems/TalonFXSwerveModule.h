// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <util/SwerveModuleConstants.h>
#include "rev/CANSparkMax.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include "rev/SparkRelativeEncoder.h"
#include <subsystems/SwerveModule.h>

class TalonFXSwerveModule : public SwerveModule {
 public:
  TalonFXSwerveModule(int module_number, SwerveModuleConstants module_constants);

  virtual void SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop) override;
  // virtual void SetSpeed(units::meters_per_second_t velcity, bool is_open_loop);
  virtual void SetAngle(frc::SwerveModuleState desired_state) override;
  // virtual void SetAngle(units::degrees angle);

  ctre::phoenix6::hardware::TalonFX GetAngleMotor();
  ctre::phoenix6::hardware::TalonFX GetDriveMotor();
  
  virtual frc::Rotation2d GetAngle() override;
  virtual units::meters_per_second_t GetVelocity() override;

  virtual frc::SwerveModulePosition GetPosition() override;

  virtual void ResetToAbsolute() override;
 private:
  ctre::phoenix6::hardware::TalonFX angle_motor;
  ctre::phoenix6::hardware::TalonFX drive_motor;

  void ConfigAngleMotor();
  void ConfigDriveMotor();
};
