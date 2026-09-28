#pragma once

#include <frc/geometry/Translation2d.h>

#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    /** A 2026 REBUILT fuel ball resting on the field. */
    class RebuiltFuelOnField : public gamepieces::GamePieceOnFieldSimulation {
    public:
        [[nodiscard]] static const GamePieceInfo& RebuiltFuelInfo();

        explicit RebuiltFuelOnField(const frc::Translation2d& initialPosition);
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
