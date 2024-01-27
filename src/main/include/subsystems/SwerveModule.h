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

class SwerveModule {
 public:
  int module_number;

  SwerveModule(int module_number, SwerveModuleConstants module_constants);
  virtual ~SwerveModule();

  void SetDesiredState(frc::SwerveModuleState desired_state, bool is_open_loop);
  virtual void SetAngle(frc::SwerveModuleState desired_state);
  // virtual void SetAngle(units::degrees angle);
  virtual void SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop);
  // virtual void SetSpeed(units::meters_per_second_t velcity, bool is_open_loop);

  units::degree_t GetCanCoderAngle();
  frc::Rotation2d GetCanCoder();

  virtual frc::Rotation2d GetAngle();
  frc::Rotation2d GetAngleSetpoint();
  virtual units::meters_per_second_t GetVelocity();

  frc::SwerveModuleState GetState();
  virtual frc::SwerveModulePosition GetPosition();

  virtual void ResetToAbsolute();
 protected:
  frc::Rotation2d angle_offset;
  frc::Rotation2d last_angle;
  ctre::phoenix6::hardware::CANcoder angle_encoder;

  frc::SimpleMotorFeedforward<units::meters> feedforward;

  void ConfigAngleEncoder();

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing
   * in appropriate scope for CTRE onboard control.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  static frc::SwerveModuleState Optimize(frc::SwerveModuleState desired_state, frc::Rotation2d current_angle);

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  static units::degree_t PlaceInAppropriate0To360Scope(units::degree_t scope_reference, units::degree_t new_angle);
};
