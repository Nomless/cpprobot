// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "swerve/drive/TalonFXSwerveDriveMotor.h"
#include <util/Conversions.h>
#include <Constants.h>

TalonFXSwerveDriveMotor::TalonFXSwerveDriveMotor(int can_id) : 
    motor(can_id),
    control(0_tps) {
}

TalonFXSwerveDriveMotor::~TalonFXSwerveDriveMotor() {}

void TalonFXSwerveDriveMotor::Set(double speed) {
  motor.Set(speed);
}

void TalonFXSwerveDriveMotor::SetVelocity(units::revolutions_per_minute_t rpm, units::volt_t feedforward) {
  motor.SetControl(control.WithVelocity(units::turns_per_second_t{rpm}).WithFeedForward(feedforward));
}

void TalonFXSwerveDriveMotor::SetLinearVelocity(units::meters_per_second_t mps, units::volt_t feedforward) {
  auto rps = Conversions::MPSToRPS(mps, SwerveConstants::kWheelCircumference);
  motor.SetControl(control.WithVelocity(rps).WithFeedForward(feedforward));
}

units::revolutions_per_minute_t TalonFXSwerveDriveMotor::GetVelocity() {
  return units::revolutions_per_minute_t{motor.GetVelocity().GetValue()};
}

units::meters_per_second_t TalonFXSwerveDriveMotor::GetLinearVelocity() {
  return Conversions::RPSToMPS(motor.GetVelocity().GetValue(), SwerveConstants::kWheelCircumference);
}

units::turn_t TalonFXSwerveDriveMotor::GetPosition() {
  return motor.GetPosition().GetValue();
}

units::meter_t TalonFXSwerveDriveMotor::GetDisplacement() {
  return Conversions::RotationsToMeters(motor.GetPosition().GetValue(), SwerveConstants::kWheelCircumference);
}

void TalonFXSwerveDriveMotor::Config() {
  ctre::phoenix6::configs::TalonFXConfiguration config;
  config.CurrentLimits
      .WithSupplyCurrentLimitEnable(SwerveConstants::kDriveEnableCurrentLimit)
      .WithSupplyCurrentLimit(SwerveConstants::kDriveContinuousCurrentLimit)
      .WithSupplyCurrentThreshold(SwerveConstants::kDrivePeakCurrentLimit)
      .WithSupplyTimeThreshold(SwerveConstants::kDrivePeakCurrentDuration);

  config.Slot0.kP = SwerveConstants::kDriveP;
  config.Slot0.kI = SwerveConstants::kDriveI;
  config.Slot0.kD = SwerveConstants::kDriveD;

  config.MotorOutput.NeutralMode = SwerveConstants::kDriveNeutralMode;
  config.Feedback.SensorToMechanismRatio = SwerveConstants::kDriveGearRatio;
  
  config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants::kOpenLoopRamp;
  config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants::kClosedLoopRamp;
  motor.GetConfigurator().Apply(config);
}

ctre::phoenix6::hardware::TalonFX* TalonFXSwerveDriveMotor::GetMotor() {
  return &motor;
}