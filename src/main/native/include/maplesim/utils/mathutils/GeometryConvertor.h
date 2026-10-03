#pragma once

#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>

namespace maplesim::utils::mathutils::GeometryConvertor {
    [[nodiscard]] wpi::math::Translation2d GetChassisSpeedsTranslationalComponent(const wpi::math::ChassisVelocities& chassisSpeeds);
} // namespace maplesim::utils::mathutils::GeometryConvertor
