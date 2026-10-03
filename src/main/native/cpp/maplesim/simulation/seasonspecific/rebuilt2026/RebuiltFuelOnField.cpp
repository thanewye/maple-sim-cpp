#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnField.h"

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    const gamepieces::GamePieceOnFieldSimulation::GamePieceInfo& RebuiltFuelOnField::RebuiltFuelInfo() {
        static const GamePieceInfo rebuiltFuelInfo{
            "Fuel", physics::Shape::Circle(wpi::units::centimeter_t{7.5}), wpi::units::centimeter_t{15}, wpi::units::pound_t{0.5}, 1.8, 5, 0.8};
        return rebuiltFuelInfo;
    }

    RebuiltFuelOnField::RebuiltFuelOnField(const wpi::math::Translation2d& initialPosition)
        : GamePieceOnFieldSimulation(RebuiltFuelInfo(), wpi::math::Pose2d{initialPosition, wpi::math::Rotation2d{}}) {}
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
