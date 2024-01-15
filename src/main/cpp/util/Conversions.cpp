// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/Conversions.h"

units::degree_t CanCoderConversions::ToDegrees(units::turn_t can_coder, double gear_ratio) {
  return units::degree_t{can_coder} / gear_ratio;
}

units::turn_t CanCoderConversions::ToCanCoder(units::degree_t degrees, double gear_ratio) {
  return units::turn_t{degrees} / gear_ratio;
}

units::degree_t TalonFXConversions::ToDegrees(units::turn_t rotations, double gear_ratio) {
  return units::degree_t{rotations} / gear_ratio;
}

units::turn_t TalonFXConversions::ToFalcon(units::degree_t degrees, double gear_ratio) {
  return units::turn_t{degrees} * gear_ratio;
}

units::revolutions_per_minute_t TalonFXConversions::ToRpm(units::turns_per_second_t rps, double gear_ratio) {
  return units::revolutions_per_minute_t{rps} / gear_ratio;
}

units::turns_per_second_t TalonFXConversions::ToFalcon(units::revolutions_per_minute_t rpm, double gear_ratio) {
  return units::turns_per_second_t{rpm} * gear_ratio;
}

units::meters_per_second_t TalonFXConversions::ToMetersPerSecond(units::turns_per_second_t rps, double circumference, double gear_ratio) {
  return units::meters_per_second_t(ToRpm(rps, gear_ratio).value() * circumference / 60);
}

units::turns_per_second_t TalonFXConversions::ToFalcon(units::meters_per_second_t velocity, double circumference, double gear_ratio) {
  auto rpm = units::revolutions_per_minute_t{(velocity.value() * 60) / circumference};
  return ToFalcon(rpm, gear_ratio);
}

units::meter_t TalonFXConversions::ToMeters(units::turn_t rotations, double circumference, double gear_ratio) {
  return units::meter_t{rotations.value() * circumference / gear_ratio};
}

units::turn_t TalonFXConversions::ToFalcon(units::meter_t meters, double circumference, double gear_ratio) {
  return units::turn_t{meters.value() / circumference * gear_ratio};
}