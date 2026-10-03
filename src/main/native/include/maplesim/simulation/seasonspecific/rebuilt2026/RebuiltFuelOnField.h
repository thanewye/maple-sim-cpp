#pragma once

#include <wpi/math/geometry/Translation2d.hpp>

#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    /** A 2026 REBUILT fuel ball resting on the field. */
    class RebuiltFuelOnField : public gamepieces::GamePieceOnFieldSimulation {
    public:
        [[nodiscard]] static const GamePieceInfo& RebuiltFuelInfo();

        explicit RebuiltFuelOnField(const wpi::math::Translation2d& initialPosition);
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
