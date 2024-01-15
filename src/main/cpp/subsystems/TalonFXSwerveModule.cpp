// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <cmath>
#include "subsystems/TalonFXSwerveModule.h"
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include "Constants.h"
#include <units/angle.h>
#include <util/Conversions.h>

TalonFXSwerveModule::TalonFXSwerveModule(int module_number, SwerveModuleConstants module_constants) : 
    SwerveModule(module_number, module_constants),
    angle_motor{module_constants.angle_motor_id},
    drive_motor{module_constants.drive_motor_id} {
  ConfigAngleMotor();
  ConfigDriveMotor();
}

void TalonFXSwerveModule::ConfigAngleMotor() {
  auto config = ctre::phoenix6::configs::TalonFXConfiguration();
  config.MotorOutput.WithInverted(SwerveConstants::kInvertAngleMotors)
      .WithNeutralMode(SwerveConstants::kAngleNeutralMode);
  config.CurrentLimits.WithSupplyCurrentLimitEnable(true)
      .WithSupplyCurrentLimit(SwerveConstants::kAngleContinuousCurrentLimit);
  config.Slot0.WithKP(SwerveConstants::kAngleP)
      .WithKI(SwerveConstants::kAngleI)
      .WithKD(SwerveConstants::kAngleD);
  angle_motor.GetConfigurator().Apply(config);
}

void TalonFXSwerveModule::ConfigDriveMotor() {
  auto config = ctre::phoenix6::configs::TalonFXConfiguration();
  config.MotorOutput.WithInverted(SwerveConstants::kInvertDriveMotors)
      .WithNeutralMode(SwerveConstants::kDriveNeutralMode);
  config.CurrentLimits.WithSupplyCurrentLimitEnable(true)
      .WithSupplyCurrentLimit(SwerveConstants::kDriveContinuousCurrentLimit);
  config.OpenLoopRamps.WithDutyCycleOpenLoopRampPeriod(SwerveConstants::kOpenLoopRamp);
  config.ClosedLoopRamps.WithVoltageClosedLoopRampPeriod(SwerveConstants::kClosedLoopRamp);

  config.Slot0.WithKP(SwerveConstants::kDriveP)
      .WithKI(SwerveConstants::kDriveI)
      .WithKD(SwerveConstants::kDriveD);
  
  drive_motor.GetConfigurator().Apply(config);
  drive_motor.SetPosition(0_tr);
}


frc::Rotation2d TalonFXSwerveModule::GetAngle() {
  return frc::Rotation2d{
    units::degree_t{angle_motor.GetPosition().GetValue()}
  };
}

units::meters_per_second_t TalonFXSwerveModule::GetVelocity() {
  return TalonFXConversions::ToMetersPerSecond(drive_motor.GetVelocity().GetValue(), SwerveConstants::kWheelCircumference, SwerveConstants::kDriveGearRatio);
}

frc::SwerveModulePosition TalonFXSwerveModule::GetPosition() {
  return frc::SwerveModulePosition{
    TalonFXConversions::ToMeters(drive_motor.GetPosition().GetValue(), SwerveConstants::kWheelCircumference, SwerveConstants::kDriveGearRatio),
    GetAngle()
  };
}

void TalonFXSwerveModule::SetAngle(frc::SwerveModuleState desired_state) {
  frc::Rotation2d angle = (std::fabs(desired_state.speed.value()) <= SwerveConstants::kMaxSpeed * 0.01) ? last_angle : desired_state.angle;
  ctre::phoenix6::controls::PositionVoltage control{TalonFXConversions::ToFalcon(angle.Degrees())};
  control.WithFeedForward(units::volt_t{SwerveConstants::kAngleF});
  angle_motor.SetControl(control);
  // angle_controller.SetReference(angle.Degrees().value(), rev::CANSparkMax::ControlType::kPosition);
  last_angle = angle;
}

void TalonFXSwerveModule::SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop) {
  if (is_open_loop) {
    double percent_output = desired_state.speed.value() / SwerveConstants::kMaxSpeed;
    drive_motor.Set(percent_output);
  }
  else {
    ctre::phoenix6::controls::VelocityVoltage control{
      TalonFXConversions::ToFalcon(desired_state.speed, SwerveConstants::kWheelCircumference, SwerveConstants::kDriveGearRatio)
    };
    control.WithFeedForward(units::volt_t{SwerveConstants::kDriveF});
    drive_motor.SetControl(control);
  }
}