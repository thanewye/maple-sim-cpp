#include "pch.h"

#include "maplesim/simulation/seasonspecific/evergreen/ArenaEvergreen.h"

#include <frc/Errors.h>
#include <frc/geometry/Translation2d.h>
#include <units/length.h>

namespace maplesim::simulation::seasonspecific::evergreen {
    namespace {
        constexpr units::meter_t kFieldLength{17.548};
        constexpr units::meter_t kFieldWidth{8.052};
        constexpr units::meter_t kEndWallStartY{1.270};
        constexpr units::meter_t kEndWallEndY{6.782};
        constexpr units::meter_t kSideWallInset{1.672};
        constexpr units::meter_t kZero{0.0};
        constexpr units::meter_t kUpperWallFirstEndX{11.0};
        constexpr units::meter_t kUpperWallSecondStartX{12.0};
        constexpr units::meter_t kLowerWallFirstEndX{5.8};
        constexpr units::meter_t kLowerWallSecondStartX{6.3};
    } // namespace

    ArenaEvergreen::EvergreenFieldObstacleMap::EvergreenFieldObstacleMap(bool withWalls) {
        if (!withWalls) return;

        AddBorderLine(frc::Translation2d{kZero, kEndWallStartY}, frc::Translation2d{kZero, kEndWallEndY});
        AddBorderLine(frc::Translation2d{kFieldLength, kEndWallStartY}, frc::Translation2d{kFieldLength, kEndWallEndY});
        AddBorderLine(frc::Translation2d{kSideWallInset, kFieldWidth}, frc::Translation2d{kUpperWallFirstEndX, kFieldWidth});
        AddBorderLine(frc::Translation2d{kUpperWallSecondStartX, kFieldWidth}, frc::Translation2d{kFieldLength - kSideWallInset, kFieldWidth});
        AddBorderLine(frc::Translation2d{kSideWallInset, kZero}, frc::Translation2d{kLowerWallFirstEndX, kZero});
        AddBorderLine(frc::Translation2d{kLowerWallSecondStartX, kZero}, frc::Translation2d{kFieldLength - kSideWallInset, kZero});
    }

    ArenaEvergreen::ArenaEvergreen(bool withWalls)
        : SimulatedArena(EvergreenFieldObstacleMap{withWalls}) {}

    void ArenaEvergreen::PlaceGamePiecesOnField() {
        FRC_ReportError(frc::err::Error, "Evergreen doesn't have game pieces.");
    }
} // namespace maplesim::simulation::seasonspecific::evergreen
