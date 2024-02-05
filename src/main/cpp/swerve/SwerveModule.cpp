// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include "swerve/SwerveModule.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include "Constants.h"
#include <util/Conversions.h>
#include <units/velocity.h>
#include <swerve/drive/TalonFXSwerveDriveMotor.h>

SwerveModule::SwerveModule(int module_number, SwerveAngleMotor* angle_motor, SwerveDriveMotor* drive_motor, SwerveModuleConstants module_constants) : 
    module_number(module_number), 
    angle_offset(module_constants.angle_offset),
    last_angle(module_constants.angle_offset),
    angle_encoder{module_constants.cancoder_id},
    angle_motor(angle_motor),
    drive_motor(drive_motor),
    feedforward{SwerveConstants::kDriveS, SwerveConstants::kDriveV, SwerveConstants::kDriveA} {
  ConfigAngleEncoder();
  angle_motor->Config();
  drive_motor->Config();

  ResetToAbsolute();

  last_angle = GetState().angle;
}

SwerveModule::~SwerveModule() {
  delete angle_motor;
  delete drive_motor;
}

void SwerveModule::ConfigAngleEncoder() {
  auto can_coder_config = ctre::phoenix6::configs::CANcoderConfiguration{};
  can_coder_config.MagnetSensor.WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1);
  angle_encoder.GetConfigurator().Apply(can_coder_config);
}

units::degree_t SwerveModule::GetCanCoderAngle() {
  return angle_encoder.GetAbsolutePosition().GetValue();
}

frc::Rotation2d SwerveModule::GetCanCoder() {
  return frc::Rotation2d{GetCanCoderAngle()};
}

frc::Rotation2d SwerveModule::GetAngle() {
  return frc::Rotation2d{units::degree_t{angle_motor->GetPosition()}};
}

frc::Rotation2d SwerveModule::GetAngleSetpoint() {
  return last_angle;
}

units::meters_per_second_t SwerveModule::GetVelocity() {
  return drive_motor->GetLinearVelocity();
}

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState{
    GetVelocity(),
    GetAngle()
  };
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
    drive_motor->GetDisplacement(),
    GetAngle()
  };
}

void SwerveModule::SetAngle(frc::SwerveModuleState desired_state) {
  angle_motor->SetPosition(desired_state.angle.Degrees());
}

void SwerveModule::SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop) {
  if (is_open_loop) {
    auto out = desired_state.speed / SwerveConstants::kMaxSpeed;
    drive_motor->Set(out.value());
  }
  else {
    drive_motor->SetLinearVelocity(desired_state.speed, feedforward.Calculate(desired_state.speed));
  }
}

void SwerveModule::SetDesiredState(frc::SwerveModuleState desired_state, bool is_open_loop) {
  desired_state = SwerveModule::Optimize(desired_state, GetAngle());
  SetAngle(desired_state);
  SetSpeed(desired_state, is_open_loop);
}

void SwerveModule::ResetToAbsolute() {
  auto absolute_position = GetCanCoderAngle() - angle_offset.Degrees();
  angle_motor->SetSensorPosition(absolute_position);
}

frc::SwerveModuleState SwerveModule::Optimize(frc::SwerveModuleState desired_state, frc::Rotation2d current_angle) {
  auto target_angle = PlaceInAppropriate0To360Scope(current_angle.Degrees(), desired_state.angle.Degrees());
  auto target_speed = desired_state.speed;
  auto delta = target_angle - current_angle.Degrees();
  if (std::fabs(delta.value()) > 90) {
    target_speed = -target_speed;
    target_angle = delta > 90_deg ? (target_angle -= 180_deg) : (target_angle += 180_deg);
  }
  return frc::SwerveModuleState{target_speed, frc::Rotation2d{target_angle}};
}

units::degree_t SwerveModule::PlaceInAppropriate0To360Scope(units::degree_t scope_reference, units::degree_t new_angle) {
  units::degree_t lower_bound;
  units::degree_t upper_bound;
  units::degree_t lower_offset = units::degree_t{std::fmod(scope_reference.value(), 360)};
  if (lower_offset >= 0_deg) {
    lower_bound = scope_reference - lower_offset;
    upper_bound = scope_reference + (360_deg - lower_offset);
  }
  else {
    upper_bound = scope_reference - lower_offset;
    lower_bound = scope_reference - (360_deg + lower_offset);
  }
  while (new_angle < lower_bound) {
    new_angle += 360_deg;
  }
  while (new_angle > upper_bound) {
    new_angle -= 360_deg;
  }
  if (new_angle - scope_reference > 180_deg) {
    new_angle -= 360_deg;
  }
  else if (new_angle - scope_reference < -180_deg) {
    new_angle += 360_deg;
  }
  return new_angle;
}