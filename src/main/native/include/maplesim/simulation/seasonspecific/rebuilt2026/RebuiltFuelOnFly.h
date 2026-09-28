#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    /** A launched 2026 REBUILT fuel ball that becomes on-field fuel after touching the ground. */
    class RebuiltFuelOnFly : public gamepieces::GamePieceProjectile {
    public:
        RebuiltFuelOnFly(const frc::Translation2d& robotPosition, const frc::Translation2d& shooterPositionOnRobot,
                         const frc::ChassisSpeeds& chassisSpeedsFieldRelative, const frc::Rotation2d& shooterFacing, units::meter_t initialHeight,
                         units::meters_per_second_t launchingSpeed, units::radian_t shooterAngle);
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
