#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/velocity.hpp>

namespace maplesim::utils::mathutils::SwerveStateProjection {
    /** Projects the module speed onto the direction the module is currently facing. */
    [[nodiscard]] wpi::units::meters_per_second_t Project(const wpi::math::SwerveModuleVelocity& swerveSpeed, const wpi::math::Rotation2d& currentSwerveFacing);
} // namespace maplesim::utils::mathutils::SwerveStateProjection
