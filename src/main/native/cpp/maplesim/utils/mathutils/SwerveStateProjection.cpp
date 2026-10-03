#include "pch.h"

#include "maplesim/utils/mathutils/SwerveStateProjection.h"

#include <cmath>

namespace maplesim::utils::mathutils::SwerveStateProjection {
    wpi::units::meters_per_second_t Project(const wpi::math::SwerveModuleVelocity& swerveSpeed, const wpi::math::Rotation2d& currentSwerveFacing) {
        const wpi::math::Rotation2d swerveModuleAngle = swerveSpeed.angle;
        const double cosTheta = std::cos((swerveModuleAngle - currentSwerveFacing).Radians().value());
        return swerveSpeed.velocity * cosTheta;
    }
} // namespace maplesim::utils::mathutils::SwerveStateProjection
