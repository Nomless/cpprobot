// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <subsystems/SwerveModule.h>
#include <subsystems/TalonFXSwerveModule.h>
#include "rev/CANSparkMax.h"
#include <frc/filter/SlewRateLimiter.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <AHRS.h>

// template<size_t ModuleCount>
class Swerve : public frc2::SubsystemBase {
 public:
  std::vector<SwerveModule*> modules;

  Swerve();

  ~Swerve();

  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states, bool is_open_loop = false);
  void Drive(frc::Translation2d translation, units::degrees_per_second_t rotation, bool is_open_loop, bool is_rate_limited = false);

  frc::Rotation2d GetYaw();
  void ZeroGyro();

  frc::Pose2d GetPose();
  void ResetOdometry(frc::Pose2d pose = frc::Pose2d{});
  
  wpi::array<frc::SwerveModuleState, 4> GetModuleStates();
  wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();

  bool IsBraking();
  void ToggleBrake();

  void ResetModulesToAbsolute();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  AHRS gyro;
  frc::SwerveDriveKinematics<4> kinematics;
  frc::SwerveDrivePoseEstimator<4> pose_estimator;
  bool braking;
  frc::SlewRateLimiter<units::meters> limiter_x, limiter_y;
};
