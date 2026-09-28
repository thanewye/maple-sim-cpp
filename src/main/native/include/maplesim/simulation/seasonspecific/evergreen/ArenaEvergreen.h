#pragma once

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation::seasonspecific::evergreen {
    class ArenaEvergreen final : public SimulatedArena {
    public:
        class EvergreenFieldObstacleMap final : public FieldMap {
        public:
            explicit EvergreenFieldObstacleMap(bool withWalls = false);
        };

        explicit ArenaEvergreen(bool withWalls);

        void PlaceGamePiecesOnField() override;
    };
} // namespace maplesim::simulation::seasonspecific::evergreen
