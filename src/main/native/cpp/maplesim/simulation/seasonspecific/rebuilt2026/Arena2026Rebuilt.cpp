#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"

#include <memory>
#include <optional>
#include <random>

#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/math/geometry/Pose2d.hpp>

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnField.h"
#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltFuelOnFly.h"
#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltHub.h"
#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltOutpost.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    namespace {
        [[nodiscard]] std::mt19937_64& GetRandom() {
            static std::mt19937_64 random{std::random_device{}()};
            return random;
        }

        [[nodiscard]] double MathRandom() {
            return std::uniform_real_distribution<double>{0.0, 1.0}(GetRandom());
        }

        [[nodiscard]] wpi::math::Pose2d MetersPose(double x, double y) {
            return wpi::math::Pose2d{wpi::units::meter_t{x}, wpi::units::meter_t{y}, wpi::math::Rotation2d{}};
        }

        [[nodiscard]] wpi::math::Translation2d FuelGridOffset(int x, int y) {
            return wpi::math::Translation2d{wpi::units::inch_t{5.991 * x}, wpi::units::inch_t{5.95 * y}};
        }
    } // namespace

    Arena2026Rebuilt::RebuiltFieldObstaclesMap::RebuiltFieldObstaclesMap(bool addRampCollider) {
        AddBorderLine(wpi::math::Translation2d{wpi::units::meter_t{kFieldXMin}, wpi::units::meter_t{kFieldYMin}},
                      wpi::math::Translation2d{wpi::units::meter_t{kFieldXMin}, wpi::units::meter_t{kFieldYMax}});

        AddBorderLine(wpi::math::Translation2d{wpi::units::meter_t{kFieldXMax}, wpi::units::meter_t{kFieldYMin}},
                      wpi::math::Translation2d{wpi::units::meter_t{kFieldXMax}, wpi::units::meter_t{kFieldYMax}});

        AddBorderLine(wpi::math::Translation2d{wpi::units::meter_t{kFieldXMin}, wpi::units::meter_t{kFieldYMin}},
                      wpi::math::Translation2d{wpi::units::meter_t{kFieldXMax}, wpi::units::meter_t{kFieldYMin}});

        AddBorderLine(wpi::math::Translation2d{wpi::units::meter_t{kFieldXMin}, wpi::units::meter_t{kFieldYMax}},
                      wpi::math::Translation2d{wpi::units::meter_t{kFieldXMax}, wpi::units::meter_t{kFieldYMax}});

        AddRectangularObstacle(kUprightXLen, kUprightYLen, MetersPose(kUprightOffsetFromEndWall, kUprightOffsetFromSideWall));
        AddRectangularObstacle(kUprightXLen, kUprightYLen, MetersPose(kUprightOffsetFromEndWall, kUprightOffsetFromSideWall + kUprightYSpacing));

        AddRectangularObstacle(kUprightXLen, kUprightYLen, MetersPose(kFieldXMax - kUprightOffsetFromEndWall, kFieldYMax - kUprightOffsetFromSideWall));
        AddRectangularObstacle(kUprightXLen, kUprightYLen,
                               MetersPose(kFieldXMax - kUprightOffsetFromEndWall, kFieldYMax - kUprightOffsetFromSideWall - kUprightYSpacing));

        AddRectangularObstacle(kTrenchWallXLen, kTrenchWallYLen, MetersPose(kTrenchWallOffsetFromEndWall, kTrenchWallOffsetFromSideWall));
        AddRectangularObstacle(kTrenchWallXLen, kTrenchWallYLen, MetersPose(kTrenchWallOffsetFromEndWall, kFieldYMax - kTrenchWallOffsetFromSideWall));

        AddRectangularObstacle(kTrenchWallXLen, kTrenchWallYLen, MetersPose(kFieldXMax - kTrenchWallOffsetFromEndWall, kTrenchWallOffsetFromSideWall));
        AddRectangularObstacle(kTrenchWallXLen, kTrenchWallYLen,
                               MetersPose(kFieldXMax - kTrenchWallOffsetFromEndWall, kFieldYMax - kTrenchWallOffsetFromSideWall));

        if (addRampCollider) {
            AddRectangularObstacle(kHubXLen, kHubYLen + 2.0 * kHubRampLength, MetersPose(kHubX, kHubY));

            AddRectangularObstacle(kHubXLen, kHubYLen + 2.0 * kHubRampLength, MetersPose(kFieldXMax - kHubX, kHubY));
        } else {
            AddRectangularObstacle(kHubXLen, kHubYLen, MetersPose(kHubX, kHubY));

            AddRectangularObstacle(kHubXLen, kHubYLen, MetersPose(kFieldXMax - kHubX, kHubY));
        }
    }

    Arena2026Rebuilt::Arena2026Rebuilt()
        : Arena2026Rebuilt(true) {}

    Arena2026Rebuilt::Arena2026Rebuilt(bool addRampCollider)
        : SimulatedArena(RebuiltFieldObstaclesMap{addRampCollider})
        , blueIsOnClock_(MathRandom() < 0.5)
        , phaseClockPublisher_(genericInfoTable->GetDoubleTopic("Time left in current phase").Publish())
        , redActivePublisher_(redTable->GetBooleanTopic("Red is active").Publish())
        , blueActivePublisher_(blueTable->GetBooleanTopic("Blue is active").Publish()) {
        blueHub_ = &AddCustomSimulation(std::make_unique<RebuiltHub>(*this, true));

        redHub_ = &AddCustomSimulation(std::make_unique<RebuiltHub>(*this, false));

        blueOutpost_ = &AddCustomSimulation(std::make_unique<RebuiltOutpost>(*this, true));

        redOutpost_ = &AddCustomSimulation(std::make_unique<RebuiltOutpost>(*this, false));
    }

    Arena2026Rebuilt::~Arena2026Rebuilt() = default;

    double Arena2026Rebuilt::RandomInRange(double variance) {
        return (MathRandom() - 0.5) * variance;
    }

    void Arena2026Rebuilt::SetSeed(std::uint64_t seed) {
        GetRandom().seed(seed);
    }

    void Arena2026Rebuilt::AddPieceWithVariance(const wpi::math::Translation2d& piecePose, const wpi::math::Rotation2d& yaw, wpi::units::meter_t height,
                                                wpi::units::meters_per_second_t speed, wpi::units::radian_t pitch, double xVariance, double yVariance,
                                                double yawVariance, double speedVariance, double pitchVariance) {
        AddGamePieceProjectile(std::make_unique<RebuiltFuelOnFly>(
            piecePose + wpi::math::Translation2d{wpi::units::meter_t{RandomInRange(xVariance)}, wpi::units::meter_t{RandomInRange(yVariance)}},
            wpi::math::Translation2d{}, wpi::math::ChassisVelocities{}, yaw + wpi::math::Rotation2d{wpi::units::degree_t{RandomInRange(yawVariance)}}, height,
            speed + wpi::units::meters_per_second_t{RandomInRange(speedVariance)},
            wpi::units::degree_t{wpi::units::degree_t{pitch}.value() + RandomInRange(pitchVariance)}));
    }

    void Arena2026Rebuilt::PlaceGamePiecesOnField() {
        blueOutpost_->Reset();
        redOutpost_->Reset();

        for (int x = 0; x < 12; x += 1) {
            for (int y = 0; y < 30; y += isInEfficiencyMode_ ? 3 : 1) {
                AddGamePiece(std::make_unique<RebuiltFuelOnField>(kCenterPieceBottomRightCorner + FuelGridOffset(x, y)));
            }
        }

        const std::optional<wpi::Alliance> alliance = wpi::MatchState::GetAlliance();
        const bool isOnBlue = alliance.has_value() && alliance.value() == wpi::Alliance::BLUE;

        if (isOnBlue || !isInEfficiencyMode_) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    AddGamePiece(std::make_unique<RebuiltFuelOnField>(kBlueDepotBottomRightCorner + FuelGridOffset(x, y)));
                }
            }
        }

        if (!isOnBlue || !isInEfficiencyMode_) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 6; y++) {
                    AddGamePiece(std::make_unique<RebuiltFuelOnField>(kRedDepotBottomRightCorner + FuelGridOffset(x, y)));
                }
            }
        }

        SetupValueForMatchBreakdown("CurrentFuelInOutpost");
        SetupValueForMatchBreakdown("TotalFuelInOutpost");
        SetupValueForMatchBreakdown("TotalFuelInHub");
        SetupValueForMatchBreakdown("WastedFuel");
    }

    std::vector<wpi::math::Pose3d> Arena2026Rebuilt::GetGamePiecesPosesByType(const std::string& type) const {
        std::vector<wpi::math::Pose3d> poses = SimulatedArena::GetGamePiecesPosesByType(type);

        blueOutpost_->Draw(poses);
        redOutpost_->Draw(poses);

        return poses;
    }

    void Arena2026Rebuilt::SimulationSubTick(int tickNum) {
        if (shouldClock_ && !wpi::RobotState::IsAutonomous() && wpi::RobotState::IsEnabled()) {
            if (matchClock_.Get().value() >= nextClockSwapTime_) {
                nextClockSwapTime_ = matchClock_.Get().value() + 25;
                blueIsOnClock_ = !blueIsOnClock_;
            }
            phaseClockPublisher_.Set(nextClockSwapTime_ - matchClock_.Get().value());
        } else {
            phaseClockPublisher_.Set(25);
        }

        SimulatedArena::SimulationSubTick(tickNum);

        blueActivePublisher_.Set(IsActive(true));
        redActivePublisher_.Set(IsActive(false));
    }

    bool Arena2026Rebuilt::IsActive(bool isBlue) const {
        if (isBlue) return blueIsOnClock_ || wpi::RobotState::IsAutonomous() || !shouldClock_;
        return !blueIsOnClock_ || wpi::RobotState::IsAutonomous() || !shouldClock_;
    }

    void Arena2026Rebuilt::SetShouldRunClock(bool shouldRunClock) {
        shouldClock_ = shouldRunClock;
    }

    void Arena2026Rebuilt::OutpostDump(bool isBlue) {
        (isBlue ? blueOutpost_ : redOutpost_)->Dump();
    }

    void Arena2026Rebuilt::OutpostThrowForGoal(bool isBlue) {
        (isBlue ? blueOutpost_ : redOutpost_)->ThrowForGoal();
    }

    void Arena2026Rebuilt::OutpostThrow(bool isBlue, const wpi::math::Rotation2d& throwYaw, wpi::units::radian_t throwPitch,
                                        wpi::units::meters_per_second_t speed) {
        (isBlue ? blueOutpost_ : redOutpost_)->ThrowFuel(throwYaw, throwPitch, speed);
    }

    void Arena2026Rebuilt::SetEfficiencyMode(bool efficiencyMode) {
        isInEfficiencyMode_ = efficiencyMode;
    }
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
