#include "pch.h"

#include "maplesim/simulation/seasonspecific/evergreen/ArenaEvergreen.h"

#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/system/Errors.hpp>
#include <wpi/units/length.hpp>

namespace maplesim::simulation::seasonspecific::evergreen {
    namespace {
        constexpr wpi::units::meter_t kFieldLength{17.548};
        constexpr wpi::units::meter_t kFieldWidth{8.052};
        constexpr wpi::units::meter_t kEndWallStartY{1.270};
        constexpr wpi::units::meter_t kEndWallEndY{6.782};
        constexpr wpi::units::meter_t kSideWallInset{1.672};
        constexpr wpi::units::meter_t kZero{0.0};
        constexpr wpi::units::meter_t kUpperWallFirstEndX{11.0};
        constexpr wpi::units::meter_t kUpperWallSecondStartX{12.0};
        constexpr wpi::units::meter_t kLowerWallFirstEndX{5.8};
        constexpr wpi::units::meter_t kLowerWallSecondStartX{6.3};
    } // namespace

    ArenaEvergreen::EvergreenFieldObstacleMap::EvergreenFieldObstacleMap(bool withWalls) {
        if (!withWalls) return;

        AddBorderLine(wpi::math::Translation2d{kZero, kEndWallStartY}, wpi::math::Translation2d{kZero, kEndWallEndY});
        AddBorderLine(wpi::math::Translation2d{kFieldLength, kEndWallStartY}, wpi::math::Translation2d{kFieldLength, kEndWallEndY});
        AddBorderLine(wpi::math::Translation2d{kSideWallInset, kFieldWidth}, wpi::math::Translation2d{kUpperWallFirstEndX, kFieldWidth});
        AddBorderLine(wpi::math::Translation2d{kUpperWallSecondStartX, kFieldWidth}, wpi::math::Translation2d{kFieldLength - kSideWallInset, kFieldWidth});
        AddBorderLine(wpi::math::Translation2d{kSideWallInset, kZero}, wpi::math::Translation2d{kLowerWallFirstEndX, kZero});
        AddBorderLine(wpi::math::Translation2d{kLowerWallSecondStartX, kZero}, wpi::math::Translation2d{kFieldLength - kSideWallInset, kZero});
    }

    ArenaEvergreen::ArenaEvergreen(bool withWalls)
        : SimulatedArena(EvergreenFieldObstacleMap{withWalls}) {}

    void ArenaEvergreen::PlaceGamePiecesOnField() {
        WPILIB_ReportError(wpi::err::Error, "Evergreen doesn't have game pieces.");
    }
} // namespace maplesim::simulation::seasonspecific::evergreen
