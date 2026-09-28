#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    class RebuiltHub;
    class RebuiltOutpost;

    /** The 2026 REBUILT field with its hubs, outposts, fuel layout and alternating hub-activity clock. */
    class Arena2026Rebuilt : public SimulatedArena {
    public:
        /** The field walls, hub uprights, trench walls and hubs, optionally with the hub ramps as solid colliders. */
        class RebuiltFieldObstaclesMap final : public FieldMap {
        public:
            explicit RebuiltFieldObstaclesMap(bool addRampCollider);

        private:
            static constexpr double kFieldXMin = 0.00000;
            static constexpr double kFieldXMax = 16.54105;
            static constexpr double kFieldYMin = 0.00000;
            static constexpr double kFieldYMax = 8.06926;

            static constexpr double kHubXLen = 1.19380;
            static constexpr double kHubYLen = 1.19380;
            static constexpr double kHubX = 4.625594;
            static constexpr double kHubY = 4.03463;
            static constexpr double kHubRampLength = 73.0 * 0.0254;

            static constexpr double kUprightXLen = 3.5 * 0.0254;
            static constexpr double kUprightYLen = 1.5 * 0.0254;
            static constexpr double kUprightOffsetFromEndWall = 1.06204;
            static constexpr double kUprightOffsetFromSideWall = 3.31524;
            static constexpr double kUprightYSpacing = 33.75 * 0.0254;

            static constexpr double kTrenchWallYLen = 12.0 * 0.0254;
            static constexpr double kTrenchWallXLen = 47.0 * 0.0254;
            static constexpr double kTrenchWallOffsetFromEndWall = 4.61769;
            static constexpr double kTrenchWallOffsetFromSideWall = 1.43113;
        };

        Arena2026Rebuilt();
        explicit Arena2026Rebuilt(bool addRampCollider);
        ~Arena2026Rebuilt() override;

        /** Uniform in [-variance / 2, variance / 2). */
        [[nodiscard]] static double RandomInRange(double variance);
        /** Test-only: reseeds the generator behind RandomInRange and the initial clock side so scenarios are reproducible. */
        static void SetSeed(std::uint64_t seed);

        void AddPieceWithVariance(const frc::Translation2d& piecePose, const frc::Rotation2d& yaw, units::meter_t height, units::meters_per_second_t speed,
                                  units::radian_t pitch, double xVariance, double yVariance, double yawVariance, double speedVariance, double pitchVariance);

        void PlaceGamePiecesOnField() override;
        [[nodiscard]] std::vector<frc::Pose3d> GetGamePiecesPosesByType(const std::string& type) const override;
        void SimulationSubTick(int tickNum) override;

        [[nodiscard]] bool IsActive(bool isBlue) const;
        void SetShouldRunClock(bool shouldRunClock);

        void OutpostDump(bool isBlue);
        void OutpostThrowForGoal(bool isBlue);
        void OutpostThrow(bool isBlue, const frc::Rotation2d& throwYaw, units::radian_t throwPitch, units::meters_per_second_t speed);

        void SetEfficiencyMode(bool efficiencyMode);
        [[nodiscard]] bool GetEfficiencyMode() const { return isInEfficiencyMode_; }

    protected:
        static constexpr frc::Translation2d kCenterPieceBottomRightCorner{units::meter_t{7.35737}, units::meter_t{1.724406}};
        static constexpr frc::Translation2d kRedDepotBottomRightCorner{units::meter_t{0.02}, units::meter_t{5.53}};
        static constexpr frc::Translation2d kBlueDepotBottomRightCorner{units::meter_t{16.0274}, units::meter_t{1.646936}};

        bool shouldClock_ = true;

        double nextClockSwapTime_ = 0;
        bool blueIsOnClock_;

        nt::DoublePublisher phaseClockPublisher_;
        nt::BooleanPublisher redActivePublisher_;
        nt::BooleanPublisher blueActivePublisher_;

        RebuiltHub* blueHub_;
        RebuiltHub* redHub_;

        RebuiltOutpost* blueOutpost_;
        RebuiltOutpost* redOutpost_;

        bool isInEfficiencyMode_ = true;
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
