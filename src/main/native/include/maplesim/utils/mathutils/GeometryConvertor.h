#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

namespace maplesim::utils::mathutils::GeometryConvertor {
    [[nodiscard]] frc::Translation2d GetChassisSpeedsTranslationalComponent(const frc::ChassisSpeeds& chassisSpeeds);
} // namespace maplesim::utils::mathutils::GeometryConvertor
