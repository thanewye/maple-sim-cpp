#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnField.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/length.h>
#include <units/mass.h>

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    const gamepieces::GamePieceOnFieldSimulation::GamePieceInfo& RebuiltFuelOnField::RebuiltFuelInfo() {
        static const GamePieceInfo rebuiltFuelInfo{
            "Fuel", physics::Shape::Circle(units::centimeter_t{7.5}), units::centimeter_t{15}, units::pound_t{0.5}, 1.8, 5, 0.8};
        return rebuiltFuelInfo;
    }

    RebuiltFuelOnField::RebuiltFuelOnField(const frc::Translation2d& initialPosition)
        : GamePieceOnFieldSimulation(RebuiltFuelInfo(), frc::Pose2d{initialPosition, frc::Rotation2d{}}) {}
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
