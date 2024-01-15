// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include "subsystems/SparkMaxSwerveModule.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include "Constants.h"

SparkMaxSwerveModule::SparkMaxSwerveModule(int module_number, SwerveModuleConstants module_constants) : 
    SwerveModule(module_number, module_constants),
    angle_motor{module_constants.angle_motor_id, rev::CANSparkMax::MotorType::kBrushless},
    drive_motor{module_constants.drive_motor_id, rev::CANSparkMax::MotorType::kBrushless},
    angle_relative_encoder{angle_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
    drive_relative_encoder{drive_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor, 42)},
    angle_controller{angle_motor.GetPIDController()},
    drive_controller{drive_motor.GetPIDController()} {
  ConfigAngleMotor();
  ConfigDriveMotor();
}

void SparkMaxSwerveModule::ConfigAngleMotor() {
  angle_motor.RestoreFactoryDefaults();
  angle_motor.SetInverted(SwerveConstants::kInvertAngleMotors);
  angle_motor.SetIdleMode(SwerveConstants::kAngleIdleMode);
  angle_motor.SetSmartCurrentLimit(SwerveConstants::kAngleContinuousCurrentLimit);
  angle_motor.EnableVoltageCompensation(12);

  angle_controller.SetP(SwerveConstants::kAngleP);
  angle_controller.SetI(SwerveConstants::kAngleI);
  angle_controller.SetD(SwerveConstants::kAngleD);
  angle_controller.SetFF(SwerveConstants::kAngleF);

  angle_motor.BurnFlash();
}

void SparkMaxSwerveModule::ConfigDriveMotor() {
  drive_motor.RestoreFactoryDefaults();
  drive_motor.SetInverted(SwerveConstants::kInvertDriveMotors);
  drive_motor.SetIdleMode(SwerveConstants::kDriveIdleMode);
  drive_motor.SetSmartCurrentLimit(SwerveConstants::kDriveContinuousCurrentLimit);
  drive_motor.SetOpenLoopRampRate(SwerveConstants::kOpenLoopRamp);
  drive_motor.SetClosedLoopRampRate(SwerveConstants::kClosedLoopRamp);
  drive_motor.EnableVoltageCompensation(12);
  drive_relative_encoder.SetPosition(0);

  drive_controller.SetP(SwerveConstants::kDriveP);
  drive_controller.SetI(SwerveConstants::kDriveI);
  drive_controller.SetD(SwerveConstants::kDriveD);
  drive_controller.SetFF(SwerveConstants::kDriveF);

  drive_motor.BurnFlash();
}


frc::Rotation2d SparkMaxSwerveModule::GetAngle() {
  return frc::Rotation2d{
    units::degree_t{angle_relative_encoder.GetPosition()}
  };
}

units::meters_per_second_t SparkMaxSwerveModule::GetVelocity() {
  return units::meters_per_second_t{drive_relative_encoder.GetVelocity()};
}

frc::SwerveModulePosition SparkMaxSwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
    units::meter_t{drive_relative_encoder.GetPosition()},
    GetAngle()
  };
}

void SparkMaxSwerveModule::SetAngle(frc::SwerveModuleState desired_state) {
  frc::Rotation2d angle = (std::fabs(desired_state.speed.value()) <= SwerveConstants::kMaxSpeed * 0.01) ? last_angle : desired_state.angle;

  angle_controller.SetReference(angle.Degrees().value(), rev::CANSparkMax::ControlType::kPosition);
  last_angle = angle;
}

void SparkMaxSwerveModule::SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop) {
  if (is_open_loop) {
    double percent_output = desired_state.speed.value() / SwerveConstants::kMaxSpeed;
    drive_motor.Set(percent_output);
  }
  else {
    double velocity = desired_state.speed.value();
    drive_controller.SetReference(velocity, rev::CANSparkMax::ControlType::kVelocity);
  }
}