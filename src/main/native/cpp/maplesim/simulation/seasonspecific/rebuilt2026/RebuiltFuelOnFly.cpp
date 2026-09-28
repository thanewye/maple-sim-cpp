#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnFly.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnField.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    RebuiltFuelOnFly::RebuiltFuelOnFly(const frc::Translation2d& robotPosition, const frc::Translation2d& shooterPositionOnRobot,
                                       const frc::ChassisSpeeds& chassisSpeedsFieldRelative, const frc::Rotation2d& shooterFacing, units::meter_t initialHeight,
                                       units::meters_per_second_t launchingSpeed, units::radian_t shooterAngle)
        : GamePieceProjectile(RebuiltFuelOnField::RebuiltFuelInfo(), robotPosition, shooterPositionOnRobot, chassisSpeedsFieldRelative, shooterFacing,
                              initialHeight, launchingSpeed, shooterAngle) {
        WithTouchGroundHeight(units::meter_t{units::inch_t{3}}.value());
        EnableBecomesGamePieceOnFieldAfterTouchGround();
    }
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
