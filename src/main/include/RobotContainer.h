// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <functional>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "subsystems/Swerve.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController xbox{
      OperatorConstants::kDriverControllerPort};

  // The robot's subsystems are defined here...
  Swerve swerve;

  // Drive Controls
  frc::SendableChooser<std::function<frc2::CommandPtr()>> auto_chooser;
  bool slow_enabled;

  void ConfigureBindings();
};
