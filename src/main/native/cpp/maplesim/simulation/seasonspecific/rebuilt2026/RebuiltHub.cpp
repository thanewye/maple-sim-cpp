#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltHub.h"

#include <cmath>
#include <random>
#include <string>

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"
#include "maplesim/utils/FieldMirroringUtils.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    namespace {
        [[nodiscard]] std::mt19937_64& GetRandom() {
            static std::mt19937_64 random{std::random_device{}()};
            return random;
        }

        [[nodiscard]] int NextShootPoseIndex() {
            return std::uniform_int_distribution<int>{0, 3}(GetRandom());
        }
    } // namespace

    const wpi::math::Translation3d& RebuiltHub::BlueHubPose() {
        static const wpi::math::Translation3d blueHubPose{wpi::units::meter_t{4.5974}, wpi::units::meter_t{4.034536}, wpi::units::meter_t{1.5748}};
        return blueHubPose;
    }

    const wpi::math::Translation3d& RebuiltHub::RedHubPose() {
        static const wpi::math::Translation3d redHubPose{wpi::units::meter_t{11.938}, wpi::units::meter_t{4.034536}, wpi::units::meter_t{1.5748}};
        return redHubPose;
    }

    const std::array<wpi::math::Pose3d, 4>& RebuiltHub::BlueShootPoses() {
        static const std::array<wpi::math::Pose3d, 4> blueShootPoses = [] {
            const auto blueShootPose = [](double yOffset, double yawDegrees) {
                return wpi::math::Pose3d{BlueHubPose() +
                                             wpi::math::Translation3d{wpi::units::meter_t{0.5969}, wpi::units::meter_t{yOffset}, wpi::units::meter_t{-0.5}},
                                         wpi::math::Rotation3d{wpi::units::degree_t{0}, wpi::units::degree_t{-15}, wpi::units::degree_t{yawDegrees}}};
            };
            return std::array<wpi::math::Pose3d, 4>{blueShootPose(0.447675, 33.75), blueShootPose(0.149225, 11.25), blueShootPose(-0.149225, 11.25),
                                                    blueShootPose(-0.447675, -33.75)};
        }();
        return blueShootPoses;
    }

    const std::array<wpi::math::Pose3d, 4>& RebuiltHub::RedShootPoses() {
        static const std::array<wpi::math::Pose3d, 4> redShootPoses = [] {
            std::array<wpi::math::Pose3d, 4> flipped;
            for (std::size_t i = 0; i < flipped.size(); i++)
                flipped[i] = utils::FieldMirroringUtils::Flip(BlueShootPoses()[i]);
            return flipped;
        }();
        return redShootPoses;
    }

    void RebuiltHub::SetSeed(std::uint64_t seed) {
        GetRandom().seed(seed);
    }

    RebuiltHub::RebuiltHub(Arena2026Rebuilt& arena, bool isBlue)
        : Goal(arena, wpi::units::inch_t{47}, wpi::units::inch_t{47}, wpi::units::inch_t{10}, "Fuel", isBlue ? BlueHubPose() : RedHubPose(), isBlue, false)
        , rebuiltArena_(arena)
        , hubPosePublisher_(wpi::nt::NetworkTableInstance::GetDefault()
                                .GetStructTopic<wpi::math::Pose3d>(std::string{"/SmartDashboard/MapleSim/Goals/"} + (isBlue ? "BlueHub" : "RedHub"))
                                .Publish()) {
        hubPosePublisher_.Set(wpi::math::Pose3d{position_, wpi::math::Rotation3d{}});
    }

    bool RebuiltHub::CheckCollision(const gamepieces::GamePiece& gamePiece) const {
        const wpi::math::Pose3d pose = gamePiece.GetPose3d();
        return std::pow(pose.X().value() - position_.X().value(), 2) + std::pow(pose.Y().value() - position_.Y().value(), 2) +
                   std::pow(pose.Z().value() - position_.Z().value(), 2) <
               std::pow(kGoalRadius, 2);
    }

    void RebuiltHub::AddPoints() {
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "WastedFuel", rebuiltArena_.IsActive(isBlue) ? 0 : 1);
        rebuiltArena_.AddToScore(isBlue, rebuiltArena_.IsActive(isBlue) ? 1 : 0);

        const wpi::math::Pose3d shootPose = isBlue ? BlueShootPoses()[NextShootPoseIndex()] : RedShootPoses()[NextShootPoseIndex()];

        rebuiltArena_.AddPieceWithVariance(shootPose.Translation().ToTranslation2d(), wpi::math::Rotation2d{shootPose.Rotation().Z()}, shootPose.Z(),
                                           wpi::units::meters_per_second_t{2}, shootPose.Rotation().Y(), 0, 0.02, 15, 0.2, 5);
    }

    void RebuiltHub::Draw([[maybe_unused]] std::vector<wpi::math::Pose3d>& drawList) const {}
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
