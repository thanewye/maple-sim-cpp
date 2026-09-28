#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltHub.h"

#include <cmath>
#include <random>
#include <string>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

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

    const frc::Translation3d& RebuiltHub::BlueHubPose() {
        static const frc::Translation3d blueHubPose{units::meter_t{4.5974}, units::meter_t{4.034536}, units::meter_t{1.5748}};
        return blueHubPose;
    }

    const frc::Translation3d& RebuiltHub::RedHubPose() {
        static const frc::Translation3d redHubPose{units::meter_t{11.938}, units::meter_t{4.034536}, units::meter_t{1.5748}};
        return redHubPose;
    }

    const std::array<frc::Pose3d, 4>& RebuiltHub::BlueShootPoses() {
        static const std::array<frc::Pose3d, 4> blueShootPoses = [] {
            const auto blueShootPose = [](double yOffset, double yawDegrees) {
                return frc::Pose3d{BlueHubPose() + frc::Translation3d{units::meter_t{0.5969}, units::meter_t{yOffset}, units::meter_t{-0.5}},
                                   frc::Rotation3d{units::degree_t{0}, units::degree_t{-15}, units::degree_t{yawDegrees}}};
            };
            return std::array<frc::Pose3d, 4>{blueShootPose(0.447675, 33.75), blueShootPose(0.149225, 11.25), blueShootPose(-0.149225, 11.25),
                                              blueShootPose(-0.447675, -33.75)};
        }();
        return blueShootPoses;
    }

    const std::array<frc::Pose3d, 4>& RebuiltHub::RedShootPoses() {
        static const std::array<frc::Pose3d, 4> redShootPoses = [] {
            std::array<frc::Pose3d, 4> flipped;
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
        : Goal(arena, units::inch_t{47}, units::inch_t{47}, units::inch_t{10}, "Fuel", isBlue ? BlueHubPose() : RedHubPose(), isBlue, false)
        , rebuiltArena_(arena)
        , hubPosePublisher_(nt::NetworkTableInstance::GetDefault()
                                .GetStructTopic<frc::Pose3d>(std::string{"/SmartDashboard/MapleSim/Goals/"} + (isBlue ? "BlueHub" : "RedHub"))
                                .Publish()) {
        hubPosePublisher_.Set(frc::Pose3d{position_, frc::Rotation3d{}});
    }

    bool RebuiltHub::CheckCollision(const gamepieces::GamePiece& gamePiece) const {
        const frc::Pose3d pose = gamePiece.GetPose3d();
        return std::pow(pose.X().value() - position_.X().value(), 2) + std::pow(pose.Y().value() - position_.Y().value(), 2) +
                   std::pow(pose.Z().value() - position_.Z().value(), 2) <
               std::pow(kGoalRadius, 2);
    }

    void RebuiltHub::AddPoints() {
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "TotalFuelInHub", 1);
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "WastedFuel", rebuiltArena_.IsActive(isBlue) ? 0 : 1);
        rebuiltArena_.AddToScore(isBlue, rebuiltArena_.IsActive(isBlue) ? 1 : 0);

        const frc::Pose3d shootPose = isBlue ? BlueShootPoses()[NextShootPoseIndex()] : RedShootPoses()[NextShootPoseIndex()];

        rebuiltArena_.AddPieceWithVariance(shootPose.Translation().ToTranslation2d(), frc::Rotation2d{shootPose.Rotation().Z()}, shootPose.Z(),
                                           units::meters_per_second_t{2}, shootPose.Rotation().Y(), 0, 0.02, 15, 0.2, 5);
    }

    void RebuiltHub::Draw([[maybe_unused]] std::vector<frc::Pose3d>& drawList) const {}
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
