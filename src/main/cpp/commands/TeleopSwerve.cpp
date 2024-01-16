// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopSwerve.h"
#include <subsystems/Swerve.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <Robot.h>
#include <functional>

TeleopSwerve::TeleopSwerve(Swerve* swerve, std::function<double()> translation, std::function<double()> strafe, std::function<double()> rotation) {
  this->swerve = swerve;
  AddRequirements(swerve);

  this->translation = translation;
  this->strafe = strafe;
  this->rotation = rotation;
}

// Called when the command is initially scheduled.
void TeleopSwerve::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TeleopSwerve::Execute() {
  if (!frc::DriverStation::IsTeleopEnabled()) return;
  frc::SmartDashboard::PutNumber("Translation", translation());
  double translation_val = frc::ApplyDeadband(translation(), OperatorConstants::kStickDeadband);
  double strafe_val = frc::ApplyDeadband(strafe(), OperatorConstants::kStickDeadband);
  double rotation_val = frc::ApplyDeadband(rotation(), OperatorConstants::kStickDeadband);

  swerve->Drive(
    frc::Translation2d{
      (translation_val * SwerveConstants::kMaxSpeed) * 1_s,
      (strafe_val * SwerveConstants::kMaxSpeed) * 1_s
    },
    units::degrees_per_second_t{rotation_val * SwerveConstants::kMaxAngularSpeed},
    true,
    true
  );
}

// Called once the command ends or is interrupted.
void TeleopSwerve::End(bool interrupted) {}

// Returns true when the command should end.
bool TeleopSwerve::IsFinished() {
  return false;
}
