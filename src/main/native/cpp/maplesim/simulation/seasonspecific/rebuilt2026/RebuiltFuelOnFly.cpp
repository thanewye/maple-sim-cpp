#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnFly.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnField.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    RebuiltFuelOnFly::RebuiltFuelOnFly(const wpi::math::Translation2d& robotPosition, const wpi::math::Translation2d& shooterPositionOnRobot,
                                       const wpi::math::ChassisVelocities& chassisSpeedsFieldRelative, const wpi::math::Rotation2d& shooterFacing,
                                       wpi::units::meter_t initialHeight, wpi::units::meters_per_second_t launchingSpeed, wpi::units::radian_t shooterAngle)
        : GamePieceProjectile(RebuiltFuelOnField::RebuiltFuelInfo(), robotPosition, shooterPositionOnRobot, chassisSpeedsFieldRelative, shooterFacing,
                              initialHeight, launchingSpeed, shooterAngle) {
        WithTouchGroundHeight(wpi::units::meter_t{wpi::units::inch_t{3}}.value());
        EnableBecomesGamePieceOnFieldAfterTouchGround();
    }
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
