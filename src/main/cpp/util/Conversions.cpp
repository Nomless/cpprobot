// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/Conversions.h"

units::meters_per_second_t Conversions::RPSToMPS(units::turns_per_second_t rps, units::meter_t circumference) {
  return units::meters_per_second_t{rps.value() * circumference.value()};
}

units::turns_per_second_t Conversions::MPSToRPS(units::meters_per_second_t velocity, units::meter_t circumference) {
  return units::turns_per_second_t{velocity.value() / circumference.value()};
}

units::meter_t Conversions::RotationsToMeters(units::turn_t rotations, units::meter_t circumference) {
  return units::meter_t{rotations.value() * circumference.value()};
}

units::turn_t Conversions::MetersToRotations(units::meter_t meters, units::meter_t circumference) {
  return units::turn_t{meters.value() / circumference.value()};
}