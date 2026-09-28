#include "pch.h"

#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>

#include <frc/geometry/Pose2d.h>

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation::gamepieces {
    namespace {
        constexpr double kLegacyFieldMirroringUtils2024FieldWidth = 16.54;
        constexpr double kLegacyFieldMirroringUtils2024FieldHeight = 8.21;
        constexpr std::size_t kMaxProjectilesRemovedPerUpdate = 5;

        [[nodiscard]] double HeightAtTime(double initialHeight, double initialVerticalSpeedMPS, double t) {
            return initialHeight + initialVerticalSpeedMPS * t - 1.0 / 2.0 * GamePieceProjectile::kGravity * t * t;
        }
    } // namespace

    GamePieceProjectile::GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const frc::Translation2d& robotPosition,
                                             const frc::Translation2d& shooterPositionOnRobot, const frc::ChassisSpeeds& chassisSpeedsFieldRelative,
                                             const frc::Rotation2d& shooterFacing, units::meter_t initialHeight, units::meters_per_second_t launchingSpeed,
                                             units::radian_t shooterAngle)
        : GamePieceProjectile(info, robotPosition + shooterPositionOnRobot.RotateBy(shooterFacing),
                              CalculateInitialProjectileVelocityMPS(shooterPositionOnRobot, chassisSpeedsFieldRelative, shooterFacing,
                                                                    launchingSpeed.value() * std::cos(shooterAngle.value())),
                              initialHeight.value(), launchingSpeed.value() * std::sin(shooterAngle.value()),
                              frc::Rotation3d{units::radian_t{0}, -shooterAngle, shooterFacing.Radians()}) {}

    frc::Translation2d GamePieceProjectile::CalculateInitialProjectileVelocityMPS(const frc::Translation2d& shooterPositionOnRobot,
                                                                                  const frc::ChassisSpeeds& chassisSpeeds, const frc::Rotation2d& chassisFacing,
                                                                                  double groundSpeedMPS) {
        const frc::Translation2d chassisTranslationalVelocity{units::meter_t{chassisSpeeds.vx.value()}, units::meter_t{chassisSpeeds.vy.value()}};
        const frc::Translation2d shooterGroundVelocityDueToChassisRotation =
            shooterPositionOnRobot.RotateBy(chassisFacing).RotateBy(frc::Rotation2d{units::degree_t{90}}) * chassisSpeeds.omega.value();
        const frc::Translation2d shooterGroundVelocity = chassisTranslationalVelocity + shooterGroundVelocityDueToChassisRotation;

        return shooterGroundVelocity + frc::Translation2d{units::meter_t{groundSpeedMPS}, chassisFacing};
    }

    GamePieceProjectile::GamePieceProjectile(const GamePieceOnFieldSimulation::GamePieceInfo& info, const frc::Translation2d& initialPosition,
                                             const frc::Translation2d& initialLaunchingVelocityMPS, double initialHeight, double initialVerticalSpeedMPS,
                                             const frc::Rotation3d& gamePieceRotation)
        : gamePieceType(info.type)
        , info_(info)
        , initialPosition_(initialPosition)
        , initialLaunchingVelocityMPS_(initialLaunchingVelocityMPS)
        , initialHeight_(initialHeight)
        , initialVerticalSpeedMPS_(initialVerticalSpeedMPS)
        , gamePieceRotation_(gamePieceRotation) {}

    void GamePieceProjectile::Launch() {
        constexpr int kMaxIterations = 100;
        constexpr double kStepSeconds = 0.02;
        std::vector<frc::Pose3d> trajectoryPoints;

        for (int i = 0; i < kMaxIterations; i++) {
            const double t = i * kStepSeconds;
            const frc::Translation3d currentPosition = GetPositionAtTime(t);
            trajectoryPoints.emplace_back(currentPosition, gamePieceRotation_);

            if (currentPosition.Z().value() < heightAsTouchGround_ && t * kGravity > initialVerticalSpeedMPS_) break;
            if (IsOutOfField(t)) break;
            const frc::Translation3d displacementToTarget = targetPositionSupplier_() - currentPosition;
            if (std::abs(displacementToTarget.X().value()) < tolerance_.X().value() && std::abs(displacementToTarget.Y().value()) < tolerance_.Y().value() &&
                std::abs(displacementToTarget.Z().value()) < tolerance_.Z().value()) {
                calculatedHitTargetTime_ = t;
                break;
            }
        }
        if (WillHitTarget()) projectileTrajectoryDisplayCallBackHitTarget_(trajectoryPoints);
        else projectileTrajectoryDisplayCallBackMiss_(trajectoryPoints);
        hitTargetCallBackCalled_ = false;

        launchedTimer_.Start();
    }

    bool GamePieceProjectile::HasHitGround() const {
        const double timeSinceLaunch = launchedTimer_.Get().value();
        return GetPositionAtTime(timeSinceLaunch).Z().value() <= heightAsTouchGround_ && timeSinceLaunch * kGravity > initialVerticalSpeedMPS_;
    }

    bool GamePieceProjectile::HasGoneOutOfField() const {
        return IsOutOfField(launchedTimer_.Get().value());
    }

    bool GamePieceProjectile::IsOutOfField(double time) const {
        const frc::Translation3d position = GetPositionAtTime(time);
        constexpr double kEdgeTolerance = 2;
        return position.X().value() < -kEdgeTolerance || position.X().value() > kLegacyFieldMirroringUtils2024FieldWidth + kEdgeTolerance ||
               position.Y().value() < -kEdgeTolerance || position.Y().value() > kLegacyFieldMirroringUtils2024FieldHeight + kEdgeTolerance;
    }

    bool GamePieceProjectile::HasHitTarget() const {
        return WillHitTarget() && launchedTimer_.Get().value() >= calculatedHitTargetTime_;
    }

    GamePieceProjectile& GamePieceProjectile::CleanUp() {
        projectileTrajectoryDisplayCallBackHitTarget_({});
        projectileTrajectoryDisplayCallBackMiss_({});
        return *this;
    }

    frc::Translation3d GamePieceProjectile::GetPositionAtTime(double t) const {
        const double height = HeightAtTime(initialHeight_, initialVerticalSpeedMPS_, t);
        const frc::Translation2d current2dPosition = initialPosition_ + initialLaunchingVelocityMPS_ * t;
        return frc::Translation3d{current2dPosition.X(), current2dPosition.Y(), units::meter_t{height}};
    }

    frc::Translation3d GamePieceProjectile::GetVelocityMPSAtTime(double t) const {
        const double verticalVelocityMPS = initialVerticalSpeedMPS_ - kGravity * t;
        return frc::Translation3d{initialLaunchingVelocityMPS_.X(), initialLaunchingVelocityMPS_.Y(), units::meter_t{verticalVelocityMPS}};
    }

    frc::Pose3d GamePieceProjectile::GetPose3d() const {
        return frc::Pose3d{GetPositionAtTime(launchedTimer_.Get().value()), gamePieceRotation_};
    }

    frc::Translation3d GamePieceProjectile::GetVelocity3dMPS() const {
        return GetVelocityMPSAtTime(launchedTimer_.Get().value());
    }

    void GamePieceProjectile::AddGamePieceAfterTouchGround(SimulatedArena& simulatedArena) {
        if (!becomesGamePieceOnGroundAfterTouchGround_) return;
        const double timeSinceLaunch = launchedTimer_.Get().value();
        simulatedArena.AddGamePiece(std::make_unique<GamePieceOnFieldSimulation>(
            info_,
            [halfGamePieceHeight = info_.gamePieceHeight.value() / 2, initialHeight = initialHeight_, initialVerticalSpeedMPS = initialVerticalSpeedMPS_,
             launchedTimer = launchedTimer_] {
                return std::max(halfGamePieceHeight, HeightAtTime(initialHeight, initialVerticalSpeedMPS, launchedTimer.Get().value()));
            },
            frc::Pose2d{GetPositionAtTime(timeSinceLaunch).ToTranslation2d(), frc::Rotation2d{}}, initialLaunchingVelocityMPS_));
    }

    void GamePieceProjectile::UpdateGamePieceProjectiles(SimulatedArena& simulatedArena, const std::vector<GamePieceProjectile*>& gamePieceProjectiles) {
        std::vector<GamePieceProjectile*> toRemoves;
        for (GamePieceProjectile* gamePieceProjectile : gamePieceProjectiles) {
            if ((gamePieceProjectile->HasHitTarget() || gamePieceProjectile->HasHitGround() || gamePieceProjectile->HasGoneOutOfField()) &&
                toRemoves.size() < kMaxProjectilesRemovedPerUpdate)
                toRemoves.push_back(gamePieceProjectile);
            if (gamePieceProjectile->HasHitTarget() && !gamePieceProjectile->hitTargetCallBackCalled_) {
                gamePieceProjectile->hitTargetCallBack_();
                gamePieceProjectile->hitTargetCallBackCalled_ = true;
            }
            if (gamePieceProjectile->HasHitGround()) gamePieceProjectile->AddGamePieceAfterTouchGround(simulatedArena);
        }

        for (GamePieceProjectile* toRemove : toRemoves)
            simulatedArena.RemovePiece(toRemove->CleanUp());
    }

    GamePieceProjectile& GamePieceProjectile::EnableBecomesGamePieceOnFieldAfterTouchGround() {
        becomesGamePieceOnGroundAfterTouchGround_ = true;
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::DisableBecomesGamePieceOnFieldAfterTouchGround() {
        becomesGamePieceOnGroundAfterTouchGround_ = false;
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::WithTargetPosition(std::function<frc::Translation3d()> targetPositionSupplier) {
        targetPositionSupplier_ = std::move(targetPositionSupplier);
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::WithTargetTolerance(const frc::Translation3d& tolerance) {
        tolerance_ = tolerance;
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::WithHitTargetCallBack(std::function<void()> hitTargetCallBack) {
        hitTargetCallBack_ = std::move(hitTargetCallBack);
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::WithProjectileTrajectoryDisplayCallBack(TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBack) {
        projectileTrajectoryDisplayCallBackMiss_ = projectileTrajectoryDisplayCallBack;
        projectileTrajectoryDisplayCallBackHitTarget_ = std::move(projectileTrajectoryDisplayCallBack);
        return *this;
    }

    GamePieceProjectile&
    GamePieceProjectile::WithProjectileTrajectoryDisplayCallBack(TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTarget,
                                                                 TrajectoryDisplayCallBack projectileTrajectoryDisplayCallBackHitTargetMiss) {
        projectileTrajectoryDisplayCallBackHitTarget_ = std::move(projectileTrajectoryDisplayCallBackHitTarget);
        projectileTrajectoryDisplayCallBackMiss_ = std::move(projectileTrajectoryDisplayCallBackHitTargetMiss);
        return *this;
    }

    GamePieceProjectile& GamePieceProjectile::WithTouchGroundHeight(double heightAsTouchGround) {
        heightAsTouchGround_ = heightAsTouchGround;
        return *this;
    }

    void GamePieceProjectile::SetHitTargetCallBack(std::function<void()> hitTargetCallBack) {
        hitTargetCallBack_ = std::move(hitTargetCallBack);
    }
} // namespace maplesim::simulation::gamepieces
