#pragma once

#include <functional>
#include <string>
#include <vector>

#include <frc/Timer.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include "maplesim/simulation/gamepieces/GamePiece.h"
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim::simulation {
    class SimulatedArena;
} // namespace maplesim::simulation

namespace maplesim::simulation::gamepieces {
    /** A launched game piece following a closed-form ballistic trajectory until it hits its target, the ground, or leaves the field. */
    class GamePieceProjectile : public GamePiece {
    public:
        using TrajectoryDisplayCallBack = std::function<void(const std::vector<frc::Pose3d>&)>;

        static constexpr double kGravity = 11;

        GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const frc::Translation2d& robotPosition,
                            const frc::Translation2d& shooterPositionOnRobot, const frc::ChassisSpeeds& chassisSpeedsFieldRelative,
                            const frc::Rotation2d& shooterFacing, units::meter_t initialHeight, units::meters_per_second_t launchingSpeed,
                            units::radian_t shooterAngle);
        GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const frc::Translation2d& initialPosition,
                            const frc::Translation2d& initialLaunchingVelocityMPS, double initialHeight, double initialVerticalSpeedMPS,
                            const frc::Rotation3d& gamePieceRotation);

        /** Previews the trajectory for the display callbacks and starts the flight timer. */
        void Launch();

        [[nodiscard]] bool HasHitGround() const;
        [[nodiscard]] bool HasGoneOutOfField() const;
        [[nodiscard]] bool WillHitTarget() const { return calculatedHitTargetTime_ != -1; }
        [[nodiscard]] bool HasHitTarget() const;
        /** Clears the displayed trajectory. */
        GamePieceProjectile& CleanUp();

        [[nodiscard]] frc::Pose3d GetPose3d() const override;
        [[nodiscard]] frc::Translation3d GetVelocity3dMPS() const override;

        void AddGamePieceAfterTouchGround(SimulatedArena& simulatedArena);
        static void UpdateGamePieceProjectiles(SimulatedArena& simulatedArena, const std::vector<GamePieceProjectile*>& gamePieceProjectiles);

        GamePieceProjectile& EnableBecomesGamePieceOnFieldAfterTouchGround();
        GamePieceProjectile& DisableBecomesGamePieceOnFieldAfterTouchGround();
        GamePieceProjectile& WithTargetPosition(std::function<frc::Translation3d()> targetPositionSupplier);
        GamePieceProjectile& WithTargetTolerance(const frc::Translation3d& tolerance);
        GamePieceProjectile& WithHitTargetCallBack(std::function<void()> hitTargetCallBack);
        GamePieceProjectile& WithProjectileTrajectoryDisplayCallBack(TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBack);
        GamePieceProjectile& WithProjectileTrajectoryDisplayCallBack(TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTarget,
                                                                     TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTargetMiss);
        GamePieceProjectile& WithTouchGroundHeight(double heightAsTouchGround);

        [[nodiscard]] const std::string& GetType() const override { return gamePieceType; }
        [[nodiscard]] bool IsGrounded() const override { return false; }
        void TriggerHitTargetCallBack() override { hitTargetCallBack_(); }
        void SetHitTargetCallBack(std::function<void()> hitTargetCallBack);

        const std::string gamePieceType;

    protected:
        [[nodiscard]] frc::Translation3d GetPositionAtTime(double t) const;

        const GamePieceOnFieldSimulation::GamePieceInfo info_;
        const frc::Translation2d initialPosition_;
        const frc::Translation2d initialLaunchingVelocityMPS_;
        const double initialHeight_;
        const double initialVerticalSpeedMPS_;
        const frc::Rotation3d gamePieceRotation_;
        frc::Timer launchedTimer_;
        bool becomesGamePieceOnGroundAfterTouchGround_ = false;

    private:
        [[nodiscard]] static frc::Translation2d CalculateInitialProjectileVelocityMPS(const frc::Translation2d& shooterPositionOnRobot,
                                                                                      const frc::ChassisSpeeds& chassisSpeeds,
                                                                                      const frc::Rotation2d& chassisFacing, double groundSpeedMPS);
        [[nodiscard]] bool IsOutOfField(double time) const;
        [[nodiscard]] frc::Translation3d GetVelocityMPSAtTime(double t) const;

        TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTarget_ = [](const std::vector<frc::Pose3d>&) {};
        TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackMiss_ = [](const std::vector<frc::Pose3d>&) {};
        frc::Translation3d tolerance_{units::meter_t{0.2}, units::meter_t{0.2}, units::meter_t{0.2}};
        std::function<frc::Translation3d()> targetPositionSupplier_ = [] {
            return frc::Translation3d{units::meter_t{0}, units::meter_t{0}, units::meter_t{-100}};
        };
        std::function<void()> hitTargetCallBack_ = [] {};
        double heightAsTouchGround_ = 0.5;
        double calculatedHitTargetTime_ = -1;
        bool hitTargetCallBackCalled_ = false;
    };
} // namespace maplesim::simulation::gamepieces
