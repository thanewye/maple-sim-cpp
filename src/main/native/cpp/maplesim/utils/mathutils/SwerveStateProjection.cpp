#include "pch.h"

#include "maplesim/utils/mathutils/SwerveStateProjection.h"

#include <cmath>

namespace maplesim::utils::mathutils::SwerveStateProjection {
    units::meters_per_second_t Project(const frc::SwerveModuleState& swerveSpeed, const frc::Rotation2d& currentSwerveFacing) {
        const frc::Rotation2d swerveModuleAngle = swerveSpeed.angle;
        const double cosTheta = std::cos((swerveModuleAngle - currentSwerveFacing).Radians().value());
        return swerveSpeed.speed * cosTheta;
    }
} // namespace maplesim::utils::mathutils::SwerveStateProjection
