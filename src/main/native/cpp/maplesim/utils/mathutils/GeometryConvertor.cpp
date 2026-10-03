#include "pch.h"

#include "maplesim/utils/mathutils/GeometryConvertor.h"

#include <wpi/units/length.hpp>

namespace maplesim::utils::mathutils::GeometryConvertor {
    wpi::math::Translation2d GetChassisSpeedsTranslationalComponent(const wpi::math::ChassisVelocities& chassisSpeeds) {
        return wpi::math::Translation2d{wpi::units::meter_t{chassisSpeeds.vx.value()}, wpi::units::meter_t{chassisSpeeds.vy.value()}};
    }
} // namespace maplesim::utils::mathutils::GeometryConvertor
