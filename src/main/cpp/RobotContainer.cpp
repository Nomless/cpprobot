// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <commands/TeleopSwerve.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  slow_enabled = false;

  // Configure the button bindings
  ConfigureBindings();

  // Autos
  auto_chooser.AddOption("nothing", [this] { return frc2::InstantCommand([this] {}).ToPtr(); });
  frc::Shuffleboard::GetTab("Auto").Add(auto_chooser);
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
  swerve.SetDefaultCommand(TeleopSwerve(
    &swerve,
    [this] { return this->xbox.GetLeftY() * (this->slow_enabled ? 0.5 : 1) * 0.8; },
    [this] { return this->xbox.GetLeftX() * (this->slow_enabled ? 0.5 : 1) * 0.8; },
    [this] { return this->xbox.GetRightX() * (this->slow_enabled ? 0.5 : 1) * 0.6; }
  ));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return auto_chooser.GetSelected()();
}
