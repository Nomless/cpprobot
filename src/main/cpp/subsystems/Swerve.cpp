// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Swerve.h"
#include <subsystems/TalonFXSwerveModule.h>
#include <Constants.h>

Swerve::Swerve() {
  modules.push_back(TalonFXSwerveModule(0, SwerveConstants::Mod0::kModule));
  modules.push_back(TalonFXSwerveModule(1, SwerveConstants::Mod1::kModule));
  modules.push_back(TalonFXSwerveModule(2, SwerveConstants::Mod2::kModule));
  modules.push_back(TalonFXSwerveModule(3, SwerveConstants::Mod3::kModule));
}

// This method will be called once per scheduler run
void Swerve::Periodic() {}
