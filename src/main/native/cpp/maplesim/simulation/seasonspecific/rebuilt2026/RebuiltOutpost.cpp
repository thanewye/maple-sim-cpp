#include "pch.h"

#include "maplesim/simulation/seasonspecific/rebuilt2026/RebuiltOutpost.h"

#include <string>

#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/units/length.hpp>

#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    namespace {
        [[nodiscard]] wpi::nt::StructPublisher<wpi::math::Pose3d> PublishGoalPose(const std::string& name) {
            return wpi::nt::NetworkTableInstance::GetDefault().GetStructTopic<wpi::math::Pose3d>("/SmartDashboard/MapleSim/Goals/" + name).Publish();
        }

        [[nodiscard]] wpi::math::Translation3d MetersTranslation(double x, double y, double z) {
            return wpi::math::Translation3d{wpi::units::meter_t{x}, wpi::units::meter_t{y}, wpi::units::meter_t{z}};
        }
    } // namespace

    const wpi::math::Translation3d& RebuiltOutpost::RedOutpostPose() {
        static const wpi::math::Translation3d redOutpostPose = MetersTranslation(16.621, 7.403338, 0);
        return redOutpostPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::RedLaunchPose() {
        static const wpi::math::Translation3d redLaunchPose = MetersTranslation(16, 8.2, 0);
        return redLaunchPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::RedDumpPose() {
        static const wpi::math::Translation3d redDumpPose = MetersTranslation(16.421, 7.2, 0);
        return redDumpPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::BlueOutpostPose() {
        static const wpi::math::Translation3d blueOutpostPose = MetersTranslation(0, 0.665988, 0);
        return blueOutpostPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::BlueDumpPose() {
        static const wpi::math::Translation3d blueDumpPose = MetersTranslation(0.2, 0.665988, 0);
        return blueDumpPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::BlueLaunchPose() {
        static const wpi::math::Translation3d blueLaunchPose = MetersTranslation(0.665988, -0.254, 0);
        return blueLaunchPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::RedRenderPose() {
        static const wpi::math::Translation3d redRenderPose = MetersTranslation(16.640988, 7.7, 0.844502);
        return redRenderPose;
    }

    const wpi::math::Translation3d& RebuiltOutpost::BlueRenderPose() {
        static const wpi::math::Translation3d blueRenderPose = MetersTranslation(-0.12, 0.325, 0.844502);
        return blueRenderPose;
    }

    RebuiltOutpost::RebuiltOutpost(Arena2026Rebuilt& arena, bool isBlue)
        : Goal(arena, wpi::units::centimeter_t{3}, wpi::units::inch_t{21}, wpi::units::centimeter_t{10}, "Fuel", isBlue ? BlueOutpostPose() : RedOutpostPose(),
               isBlue, true)
        , rebuiltArena_(arena)
        , outpostPublisher_(PublishGoalPose(isBlue ? "BlueOutpost" : "RedOutpost"))
        , outpostThrowPublisher_(PublishGoalPose(isBlue ? "BlueOutpostThrow" : "RedOutpostThrow"))
        , outpostDumpPublisher_(PublishGoalPose(isBlue ? "BlueOutpostDump" : "RedOutpostDump")) {
        gamePieceCount_ = 24;

        outpostPublisher_.Set(wpi::math::Pose3d{position_, wpi::math::Rotation3d{}});
        outpostDumpPublisher_.Set(wpi::math::Pose3d{isBlue ? BlueDumpPose() : RedDumpPose(), wpi::math::Rotation3d{}});
        outpostThrowPublisher_.Set(wpi::math::Pose3d{isBlue ? BlueLaunchPose() : RedLaunchPose(), wpi::math::Rotation3d{}});
    }

    void RebuiltOutpost::AddPoints() {
        rebuiltArena_.AddValueToMatchBreakdown(isBlue, "TotalFuelInOutpost", 1);
        gamePieceCount_++;
    }

    void RebuiltOutpost::SimulationSubTick(int subTickNum) {
        Goal::SimulationSubTick(subTickNum);
        rebuiltArena_.ReplaceValueInMatchBreakDown(isBlue, "CurrentFuelInOutpost", gamePieceCount_);
    }

    void RebuiltOutpost::Draw(std::vector<wpi::math::Pose3d>& drawList) const {
        int count = 0;
        for (int col = 0; col < 5 && count < gamePieceCount_; col++) {
            for (int row = 0; row < 5 && count < gamePieceCount_; row++) {
                count++;
                if (isBlue)
                    drawList.emplace_back(BlueRenderPose() + wpi::math::Translation3d{wpi::units::inch_t{-6.0 * col}, wpi::units::inch_t{6.0 * row},
                                                                                      wpi::units::inch_t{1.3 * col}},
                                          wpi::math::Rotation3d{});
                else
                    drawList.emplace_back(RedRenderPose() + wpi::math::Translation3d{wpi::units::inch_t{6.0 * col}, wpi::units::inch_t{-6.0 * row},
                                                                                     wpi::units::inch_t{1.3 * col}},
                                          wpi::math::Rotation3d{});
            }
        }
    }

    void RebuiltOutpost::Reset() {
        gamePieceCount_ = 24;
    }

    void RebuiltOutpost::ThrowForGoal() {
        ThrowFuel(isBlue ? wpi::math::Rotation2d{wpi::units::degree_t{45}} : wpi::math::Rotation2d{wpi::units::degree_t{220}}, wpi::units::degree_t{75},
                  wpi::units::meters_per_second_t{11.2});
    }

    void RebuiltOutpost::Dump() {
        for (int i = 0; i < 24 && gamePieceCount_ > 0; i++) {
            gamePieceCount_--;
            rebuiltArena_.AddPieceWithVariance(isBlue ? BlueDumpPose().ToTranslation2d() : RedDumpPose().ToTranslation2d(), wpi::math::Rotation2d{},
                                               wpi::units::meter_t{1.7}, isBlue ? wpi::units::meters_per_second_t{2} : wpi::units::meters_per_second_t{-2},
                                               wpi::units::degree_t{0}, 0, 0.2, 5, 0.2, 5.0);
        }
    }

    void RebuiltOutpost::ThrowFuel(const wpi::math::Rotation2d& yaw, wpi::units::radian_t pitch, wpi::units::meters_per_second_t speed) {
        if (gamePieceCount_ > 0) {
            gamePieceCount_--;
            rebuiltArena_.AddPieceWithVariance(isBlue ? BlueLaunchPose().ToTranslation2d() : RedLaunchPose().ToTranslation2d(), yaw, wpi::units::meter_t{1.7},
                                               speed, pitch, 0, 0, 15, 2, 5);
        }
    }
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
