#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltOutpost.h"

#include <string>

#include <frc/geometry/Rotation3d.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>

#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    namespace {
        [[nodiscard]] nt::StructPublisher<frc::Pose3d> PublishGoalPose(const std::string& name) {
            return nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/SmartDashboard/MapleSim/Goals/" + name).Publish();
        }

        [[nodiscard]] frc::Translation3d MetersTranslation(double x, double y, double z) {
            return frc::Translation3d{units::meter_t{x}, units::meter_t{y}, units::meter_t{z}};
        }
    } // namespace

    const frc::Translation3d& RebuiltOutpost::RedOutpostPose() {
        static const frc::Translation3d redOutpostPose = MetersTranslation(16.621, 7.403338, 0);
        return redOutpostPose;
    }

    const frc::Translation3d& RebuiltOutpost::RedLaunchPose() {
        static const frc::Translation3d redLaunchPose = MetersTranslation(16, 8.2, 0);
        return redLaunchPose;
    }

    const frc::Translation3d& RebuiltOutpost::RedDumpPose() {
        static const frc::Translation3d redDumpPose = MetersTranslation(16.421, 7.2, 0);
        return redDumpPose;
    }

    const frc::Translation3d& RebuiltOutpost::BlueOutpostPose() {
        static const frc::Translation3d blueOutpostPose = MetersTranslation(0, 0.665988, 0);
        return blueOutpostPose;
    }

    const frc::Translation3d& RebuiltOutpost::BlueDumpPose() {
        static const frc::Translation3d blueDumpPose = MetersTranslation(0.2, 0.665988, 0);
        return blueDumpPose;
    }

    const frc::Translation3d& RebuiltOutpost::BlueLaunchPose() {
        static const frc::Translation3d blueLaunchPose = MetersTranslation(0.665988, -0.254, 0);
        return blueLaunchPose;
    }

    const frc::Translation3d& RebuiltOutpost::RedRenderPose() {
        static const frc::Translation3d redRenderPose = MetersTranslation(16.640988, 7.7, 0.844502);
        return redRenderPose;
    }

    const frc::Translation3d& RebuiltOutpost::BlueRenderPose() {
        static const frc::Translation3d blueRenderPose = MetersTranslation(-0.12, 0.325, 0.844502);
        return blueRenderPose;
    }

    RebuiltOutpost::RebuiltOutpost(Arena2026Rebuilt& arena, bool isBlue)
        : Goal(arena, units::centimeter_t{3}, units::inch_t{21}, units::centimeter_t{10}, "Fuel", isBlue ? BlueOutpostPose() : RedOutpostPose(), isBlue, true)
        , rebuiltArena_(arena)
        , outpostPublisher_(PublishGoalPose(isBlue ? "BlueOutpost" : "RedOutpost"))
        , outpostThrowPublisher_(PublishGoalPose(isBlue ? "BlueOutpostThrow" : "RedOutpostThrow"))
        , outpostDumpPublisher_(PublishGoalPose(isBlue ? "BlueOutpostDump" : "RedOutpostDump")) {
        gamePieceCount_ = 24;

        outpostPublisher_.Set(frc::Pose3d{position_, frc::Rotation3d{}});
        outpostDumpPublisher_.Set(frc::Pose3d{isBlue ? BlueDumpPose() : RedDumpPose(), frc::Rotation3d{}});
        outpostThrowPublisher_.Set(frc::Pose3d{isBlue ? BlueLaunchPose() : RedLaunchPose(), frc::Rotation3d{}});
    }

    void RebuiltOutpost::AddPoints() {
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "TotalFuelInOutpost", 1);
        gamePieceCount_++;
    }

    void RebuiltOutpost::SimulationSubTick(int subTickNum) {
        Goal::SimulationSubTick(subTickNum);
        rebuiltArena_.ReplaceValueInMatchBreakDown(isBlue, "CurrentFuelInOutpost", gamePieceCount_);
    }

    void RebuiltOutpost::Draw(std::vector<frc::Pose3d>& drawList) const {
        int count = 0;
        for (int col = 0; col < 5 && count < gamePieceCount_; col++) {
            for (int row = 0; row < 5 && count < gamePieceCount_; row++) {
                count++;
                if (isBlue)
                    drawList.emplace_back(BlueRenderPose() + frc::Translation3d{units::inch_t{-6.0 * col}, units::inch_t{6.0 * row}, units::inch_t{1.3 * col}},
                                          frc::Rotation3d{});
                else
                    drawList.emplace_back(RedRenderPose() + frc::Translation3d{units::inch_t{6.0 * col}, units::inch_t{-6.0 * row}, units::inch_t{1.3 * col}},
                                          frc::Rotation3d{});
            }
        }
    }

    void RebuiltOutpost::Reset() {
        gamePieceCount_ = 24;
    }

    void RebuiltOutpost::ThrowForGoal() {
        ThrowFuel(isBlue ? frc::Rotation2d{units::degree_t{45}} : frc::Rotation2d{units::degree_t{220}}, units::degree_t{75}, units::meters_per_second_t{11.2});
    }

    void RebuiltOutpost::Dump() {
        for (int i = 0; i < 24 && gamePieceCount_ > 0; i++) {
            gamePieceCount_--;
            rebuiltArena_.AddPieceWithVariance(isBlue ? BlueDumpPose().ToTranslation2d() : RedDumpPose().ToTranslation2d(), frc::Rotation2d{},
                                               units::meter_t{1.7}, isBlue ? units::meters_per_second_t{2} : units::meters_per_second_t{-2}, units::degree_t{0},
                                               0, 0.2, 5, 0.2, 5.0);
        }
    }

    void RebuiltOutpost::ThrowFuel(const frc::Rotation2d& yaw, units::radian_t pitch, units::meters_per_second_t speed) {
        if (gamePieceCount_ > 0) {
            gamePieceCount_--;
            rebuiltArena_.AddPieceWithVariance(isBlue ? BlueLaunchPose().ToTranslation2d() : RedLaunchPose().ToTranslation2d(), yaw, units::meter_t{1.7}, speed,
                                               pitch, 0, 0, 15, 2, 5);
        }
    }
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
