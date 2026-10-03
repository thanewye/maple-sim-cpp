#pragma once

#include <functional>
#include <string>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/simulation/gamepieces/GamePiece.h"
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim::simulation {
    class SimulatedArena;
} // namespace maplesim::simulation

namespace maplesim::simulation::gamepieces {
    /** A launched game piece following a closed-form ballistic trajectory until it hits its target, the ground, or leaves the field. */
    class GamePieceProjectile : public GamePiece {
    public:
        using TrajectoryDisplayCallBack = std::function<void(const std::vector<wpi::math::Pose3d>&)>;

        static constexpr double kGravity = 11;

        GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const wpi::math::Translation2d& robotPosition,
                            const wpi::math::Translation2d& shooterPositionOnRobot, const wpi::math::ChassisVelocities& chassisSpeedsFieldRelative,
                            const wpi::math::Rotation2d& shooterFacing, wpi::units::meter_t initialHeight, wpi::units::meters_per_second_t launchingSpeed,
                            wpi::units::radian_t shooterAngle);
        GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const wpi::math::Translation2d& initialPosition,
                            const wpi::math::Translation2d& initialLaunchingVelocityMPS, double initialHeight, double initialVerticalSpeedMPS,
                            const wpi::math::Rotation3d& gamePieceRotation);

        /** Previews the trajectory for the display callbacks and starts the flight timer. */
        void Launch();

        [[nodiscard]] bool HasHitGround() const;
        [[nodiscard]] bool HasGoneOutOfField() const;
        [[nodiscard]] bool WillHitTarget() const { return calculatedHitTargetTime_ != -1; }
        [[nodiscard]] bool HasHitTarget() const;
        /** Clears the displayed trajectory. */
        GamePieceProjectile& CleanUp();

        [[nodiscard]] wpi::math::Pose3d GetPose3d() const override;
        [[nodiscard]] wpi::math::Translation3d GetVelocity3dMPS() const override;

        void AddGamePieceAfterTouchGround(SimulatedArena& simulatedArena);
        static void UpdateGamePieceProjectiles(SimulatedArena& simulatedArena, const std::vector<GamePieceProjectile*>& gamePieceProjectiles);

        GamePieceProjectile& EnableBecomesGamePieceOnFieldAfterTouchGround();
        GamePieceProjectile& DisableBecomesGamePieceOnFieldAfterTouchGround();
        GamePieceProjectile& WithTargetPosition(std::function<wpi::math::Translation3d()> targetPositionSupplier);
        GamePieceProjectile& WithTargetTolerance(const wpi::math::Translation3d& tolerance);
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
        [[nodiscard]] wpi::math::Translation3d GetPositionAtTime(double t) const;

        const GamePieceOnFieldSimulation::GamePieceInfo info_;
        const wpi::math::Translation2d initialPosition_;
        const wpi::math::Translation2d initialLaunchingVelocityMPS_;
        const double initialHeight_;
        const double initialVerticalSpeedMPS_;
        const wpi::math::Rotation3d gamePieceRotation_;
        wpi::Timer launchedTimer_;
        bool becomesGamePieceOnGroundAfterTouchGround_ = false;

    private:
        [[nodiscard]] static wpi::math::Translation2d CalculateInitialProjectileVelocityMPS(const wpi::math::Translation2d& shooterPositionOnRobot,
                                                                                            const wpi::math::ChassisVelocities& chassisSpeeds,
                                                                                            const wpi::math::Rotation2d& chassisFacing, double groundSpeedMPS);
        [[nodiscard]] bool IsOutOfField(double time) const;
        [[nodiscard]] wpi::math::Translation3d GetVelocityMPSAtTime(double t) const;

        TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTarget_ = [](const std::vector<wpi::math::Pose3d>&) {};
        TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackMiss_ = [](const std::vector<wpi::math::Pose3d>&) {};
        wpi::math::Translation3d tolerance_{wpi::units::meter_t{0.2}, wpi::units::meter_t{0.2}, wpi::units::meter_t{0.2}};
        std::function<wpi::math::Translation3d()> targetPositionSupplier_ = [] {
            return wpi::math::Translation3d{wpi::units::meter_t{0}, wpi::units::meter_t{0}, wpi::units::meter_t{-100}};
        };
        std::function<void()> hitTargetCallBack_ = [] {};
        double heightAsTouchGround_ = 0.5;
        double calculatedHitTargetTime_ = -1;
        bool hitTargetCallBackCalled_ = false;
    };
} // namespace maplesim::simulation::gamepieces
