#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    /** A launched 2026 REBUILT fuel ball that becomes on-field fuel after touching the ground. */
    class RebuiltFuelOnFly : public gamepieces::GamePieceProjectile {
    public:
        RebuiltFuelOnFly(const wpi::math::Translation2d& robotPosition, const wpi::math::Translation2d& shooterPositionOnRobot,
                         const wpi::math::ChassisVelocities& chassisSpeedsFieldRelative, const wpi::math::Rotation2d& shooterFacing,
                         wpi::units::meter_t initialHeight, wpi::units::meters_per_second_t launchingSpeed, wpi::units::radian_t shooterAngle);
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
