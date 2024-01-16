// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Swerve.h"
#include <subsystems/TalonFXSwerveModule.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <Constants.h>

Swerve::Swerve() : 
    gyro{frc::SerialPort::kMXP},
    kinematics{
      frc::Translation2d{SwerveConstants::kWheelBase / 2.0, SwerveConstants::kTrackWidth / 2.0},
      frc::Translation2d{SwerveConstants::kWheelBase / 2.0, -SwerveConstants::kTrackWidth / 2.0},
      frc::Translation2d{-SwerveConstants::kWheelBase / 2.0, SwerveConstants::kTrackWidth / 2.0},
      frc::Translation2d{-SwerveConstants::kWheelBase / 2.0, -SwerveConstants::kTrackWidth / 2.0}
    },
    pose_estimator{kinematics, GetYaw(), GetModulePositions(), frc::Pose2d()},
    braking(),
    limiter_x{6_mps},
    limiter_y{6_mps} {
  modules = {
    new TalonFXSwerveModule(0, SwerveConstants::Mod0::kModule),
    new TalonFXSwerveModule(1, SwerveConstants::Mod1::kModule),
    new TalonFXSwerveModule(2, SwerveConstants::Mod2::kModule),
    new TalonFXSwerveModule(3, SwerveConstants::Mod3::kModule),
  };
  ResetModulesToAbsolute();
  ZeroGyro();
}

Swerve::~Swerve() {
  for (auto mod : modules) {
    delete mod;
  }
  modules.clear();
}

void Swerve::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> states, bool is_open_loop) {
  kinematics.DesaturateWheelSpeeds(&states, SwerveConstants::kMaxSpeed);
  for (SwerveModule* mod : modules) {
    mod->SetDesiredState(states[mod->module_number], is_open_loop);
  }
}

void Swerve::Drive(frc::Translation2d translation, units::degrees_per_second_t rotation, bool is_open_loop, bool is_rate_limited) {
  if (braking) return;
  auto states = kinematics.ToSwerveModuleStates(
    frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      (is_rate_limited ? limiter_x.Calculate(translation.X()) : translation.X()) / 1_s,
      (is_rate_limited ? limiter_y.Calculate(translation.Y()) : translation.Y()) / 1_s,
      units::radians_per_second_t{rotation},
      GetYaw()
    )
  );
  SetModuleStates(states, is_open_loop);
}

frc::Rotation2d Swerve::GetYaw() {
  return frc::Rotation2d(units::degree_t{std::fabs(gyro.GetYaw() - 180), 180});
}

void Swerve::ZeroGyro() {
  gyro.ZeroYaw();
  ResetOdometry();
}

frc::Pose2d Swerve::GetPose() {
  return pose_estimator.GetEstimatedPosition();
}

void Swerve::ResetOdometry(frc::Pose2d pose) {
  pose_estimator.ResetPosition(GetYaw(), GetModulePositions(), pose);
}

wpi::array<frc::SwerveModuleState, 4> Swerve::GetModuleStates() {
  std::array<frc::SwerveModuleState, 4> states;
  for (SwerveModule* mod : modules) {
    states[mod->module_number] = mod->GetState();
  }
  return states;
}

wpi::array<frc::SwerveModulePosition, 4> Swerve::GetModulePositions() {
  std::array<frc::SwerveModulePosition, 4> positions;
  for (SwerveModule* mod : modules) {
    positions[mod->module_number] = mod->GetPosition();
  }
  return positions;
}

bool Swerve::IsBraking() {
  return braking;
}

void Swerve::ToggleBrake() {
  if (braking) {
    SetModuleStates({
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d()),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d()),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d()),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d())
    });
  }
  else {
    SetModuleStates({
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d(-45_deg)),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d(45_deg)),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d(45_deg)),
      frc::SwerveModuleState(SwerveConstants::kMaxSpeed * 0.011, frc::Rotation2d(-45_deg))
    });
  }
  SetModuleStates({
    frc::SwerveModuleState(0_mps, frc::Rotation2d()),
    frc::SwerveModuleState(0_mps, frc::Rotation2d()),
    frc::SwerveModuleState(0_mps, frc::Rotation2d()),
    frc::SwerveModuleState(0_mps, frc::Rotation2d())
  });
  frc::SmartDashboard::PutBoolean("Is Braking", braking);
  braking = false;
}

void Swerve::ResetModulesToAbsolute() {
  for (SwerveModule* mod : modules) {
    mod->ResetToAbsolute();
  }
}

// This method will be called once per scheduler run
void Swerve::Periodic() {
  pose_estimator.Update(GetYaw(), GetModulePositions());
  frc::SmartDashboard::PutNumber("Roll", gyro.GetRoll());
  frc::SmartDashboard::PutNumber("Heading", GetYaw().Degrees().value());
  for (SwerveModule* mod : modules) {
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod->module_number) + " Cancoder (Degrees)", mod->GetCanCoderAngle().value());
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod->module_number) + " Integrated (Degrees)", mod->GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Mod " + std::to_string(mod->module_number) + " Velocity (m/s)", mod->GetVelocity().value());
  }
}
