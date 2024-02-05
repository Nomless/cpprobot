// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "swerve/angle/TalonFXSwerveAngleMotor.h"
#include <Constants.h>

TalonFXSwerveAngleMotor::TalonFXSwerveAngleMotor(int can_id) : 
    motor(can_id),
    control(0_tr) {}

TalonFXSwerveAngleMotor::~TalonFXSwerveAngleMotor() {}

inline void TalonFXSwerveAngleMotor::Set(double speed) {
  motor.Set(speed);
}

inline void TalonFXSwerveAngleMotor::SetPosition(units::turn_t rotations) {
  motor.SetControl(control.WithPosition(rotations));
}

void TalonFXSwerveAngleMotor::SetSensorPosition(units::turn_t rotations) {
  motor.SetPosition(rotations);
}

units::turn_t TalonFXSwerveAngleMotor::GetPosition() {
  return motor.GetPosition().GetValue();
}

void TalonFXSwerveAngleMotor::Config() {
  ctre::phoenix6::configs::TalonFXConfiguration config;
  config.CurrentLimits
      .WithSupplyCurrentLimitEnable(SwerveConstants::kAngleEnableCurrentLimit)
      .WithSupplyCurrentLimit(SwerveConstants::kAngleContinuousCurrentLimit)
      .WithSupplyCurrentThreshold(SwerveConstants::kAnglePeakCurrentLimit)
      .WithSupplyTimeThreshold(SwerveConstants::kAnglePeakCurrentDuration);

  config.Slot0.kP = SwerveConstants::kAngleP;
  config.Slot0.kI = SwerveConstants::kAngleI;
  config.Slot0.kD = SwerveConstants::kAngleD;

  config.MotorOutput.NeutralMode = SwerveConstants::kAngleNeutralMode;
  config.Feedback.SensorToMechanismRatio = SwerveConstants::kAngleGearRatio;
  
  motor.GetConfigurator().Apply(config);
}

ctre::phoenix6::hardware::TalonFX* TalonFXSwerveAngleMotor::GetMotor() {
  return &motor;
}