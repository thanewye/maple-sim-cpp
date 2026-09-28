#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/velocity.h>

namespace maplesim::utils::mathutils::SwerveStateProjection {
    /** Projects the module speed onto the direction the module is currently facing. */
    [[nodiscard]] units::meters_per_second_t Project(const frc::SwerveModuleState& swerveSpeed, const frc::Rotation2d& currentSwerveFacing);
} // namespace maplesim::utils::mathutils::SwerveStateProjection
