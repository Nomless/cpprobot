// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/voltage.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include "rev/CANSparkMax.h"
#include <frc/controller/SimpleMotorFeedforward.h>
#include "util/SwerveModuleConstants.h"
#include <ctre/phoenix6/signals/SpnEnums.hpp>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace SwerveConstants {

/* Drivetrain Constants */
inline constexpr double kTrackWidth = 0.5461; 
inline constexpr double kWheelBase = 0.5461; 
inline constexpr double kWheelCircumference = units::meter_t(4_in).value(); 

/* Module Gear Ratios */
inline constexpr double kAngleGearRatio = ((150.0 / 7.0) / 1.0);
inline constexpr double kDriveGearRatio = (8.14 / 1.0);

/* Inverts */
// TODO: incorrect probably
inline constexpr bool kInvertAngleMotors = true;
inline constexpr bool kInvertDriveMotors = false;
inline constexpr bool kInvertCanCoder = false;

/* Current Limiting */
inline constexpr int kAngleContinuousCurrentLimit = 40;
inline constexpr int kAnglePeakCurrentLimit = 60;
inline constexpr int kDriveContinuousCurrentLimit = 40;
inline constexpr int kDrivePeakCurrentLimit = 60;

/* Ramping */
inline constexpr double kOpenLoopRamp = 0.25;
inline constexpr double kClosedLoopRamp = 0.0;

/* Angle Motor PID Values */
inline constexpr double kAngleP = 0;
inline constexpr double kAngleI = 0;
inline constexpr double kAngleD = 0;
inline constexpr double kAngleF = 0;

/* Drive Motor PID Values */
inline constexpr double kDriveP = 0;
inline constexpr double kDriveI = 0;
inline constexpr double kDriveD = 0;
inline constexpr double kDriveF = 0;

/* Drive Motor Characterization */
// inline constexpr auto kDriveS = 0.13126_V;
// inline constexpr auto kDriveV = 2.6745_V * 1_m / 1_s;
// inline constexpr auto kDriveA = 0.24541_V * 1_m / 1_s / 1_s;

/* Swerve Profiling Values */
inline constexpr double kMaxSpeed = 2;
inline constexpr double kMaxAngularSpeed = 4;

/* Idle Modes */
inline constexpr auto kDriveIdleMode = rev::CANSparkMax::IdleMode::kBrake;
inline constexpr auto kAngleIdleMode = rev::CANSparkMax::IdleMode::kBrake;

/* Neutral Modes */
inline constexpr auto kDriveNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
inline constexpr auto kAngleNeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

/* Module Specific Constants */
/* Front Left Module - Module 0 */
namespace Mod0 {
inline constexpr int kDriveMotorId = 0;
inline constexpr int kAngleMotorId = 0;
inline constexpr int kCanCoderId = 0;
inline constexpr auto kAngleOffset = 0_deg;
inline constexpr auto kModule = SwerveModuleConstants{
    kDriveMotorId, kAngleMotorId, kCanCoderId, frc::Rotation2d{kAngleOffset}};
}

/* Front Right Module - Module 1 */
namespace Mod1 {
inline constexpr int kDriveMotorId = 0;
inline constexpr int kAngleMotorId = 0;
inline constexpr int kCanCoderId = 0;
inline constexpr auto kAngleOffset = 0_deg;
inline constexpr auto kModule = SwerveModuleConstants{
    kDriveMotorId, kAngleMotorId, kCanCoderId, frc::Rotation2d{kAngleOffset}};
}

/* Back Left Module - Module 2 */
namespace Mod2 {
inline constexpr int kDriveMotorId = 0;
inline constexpr int kAngleMotorId = 0;
inline constexpr int kCanCoderId = 0;
inline constexpr auto kAngleOffset = 0_deg;
inline constexpr auto kModule = SwerveModuleConstants{
    kDriveMotorId, kAngleMotorId, kCanCoderId, frc::Rotation2d{kAngleOffset}};
}

/* Back Right Module - Module 3 */
namespace Mod3 {
inline constexpr int kDriveMotorId = 0;
inline constexpr int kAngleMotorId = 0;
inline constexpr int kCanCoderId = 0;
inline constexpr auto kAngleOffset = 0_deg;
inline constexpr auto kModule = SwerveModuleConstants{
    kDriveMotorId, kAngleMotorId, kCanCoderId, frc::Rotation2d{kAngleOffset}};
}
}
