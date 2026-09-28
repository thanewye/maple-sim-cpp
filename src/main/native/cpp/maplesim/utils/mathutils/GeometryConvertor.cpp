#include "pch.h"

#include "maplesim/utils/mathutils/GeometryConvertor.h"

#include <units/length.h>

namespace maplesim::utils::mathutils::GeometryConvertor {
    frc::Translation2d GetChassisSpeedsTranslationalComponent(const frc::ChassisSpeeds& chassisSpeeds) {
        return frc::Translation2d{units::meter_t{chassisSpeeds.vx.value()}, units::meter_t{chassisSpeeds.vy.value()}};
    }
} // namespace maplesim::utils::mathutils::GeometryConvertor
