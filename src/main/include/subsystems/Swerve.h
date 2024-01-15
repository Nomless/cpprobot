// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <subsystems/SwerveModule.h>
#include "rev/CANSparkMax.h"

class Swerve : public frc2::SubsystemBase {
 public:
  SwerveModule modules[4];

  Swerve();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  
};
