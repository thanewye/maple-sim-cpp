#include "pch.h"

#include "maplesim/simulation/Goal.h"

#include <cmath>
#include <numbers>
#include <utility>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

namespace maplesim::simulation {
    frc::Rotation3d Goal::FlipRotation(const frc::Rotation3d& toFlip) {
        return frc::Rotation3d{units::radian_t{0}, -toFlip.Y(), toFlip.Z() + units::radian_t{std::numbers::pi}};
    }

    Goal::PositionChecker Goal::Box(const frc::Rectangle2d& xyBox, double minZMeters, double maxZMeters) {
        return [xyBox, minZMeters, maxZMeters](const frc::Translation3d& position) {
            return xyBox.Contains(position.ToTranslation2d()) && position.Z().value() >= minZMeters && position.Z().value() <= maxZMeters;
        };
    }

    Goal::RotationChecker Goal::AbsoluteAngle(const frc::Rotation3d& expectedAngle, units::degree_t tolerance) {
        return [expectedAngle, tolerance](const gamepieces::GamePiece& gamePiece) {
            const frc::Rotation3d actualRotation = gamePiece.GetPose3d().Rotation();
            const frc::Rotation3d normalDiff = actualRotation - expectedAngle;
            const frc::Rotation3d flippedDiff = FlipRotation(actualRotation) - expectedAngle;

            const units::degree_t normalAngle = frc::Rotation3d{units::radian_t{0}, normalDiff.Y(), normalDiff.Z()}.Angle();
            const units::degree_t flippedAngle = frc::Rotation3d{units::radian_t{0}, flippedDiff.Y(), flippedDiff.Z()}.Angle();

            return normalAngle < tolerance || flippedAngle < tolerance;
        };
    }

    Goal::RotationChecker Goal::PitchOnly(double expectedPitchRadians, units::radian_t tolerance) {
        return [expectedPitchRadians, tolerance](const gamepieces::GamePiece& gamePiece) {
            const double actualPitch = gamePiece.GetPose3d().Rotation().Y().value();
            return std::abs(actualPitch - expectedPitchRadians) < tolerance.value();
        };
    }

    Goal::RotationChecker Goal::AnyRotation() {
        return [](const gamepieces::GamePiece&) { return true; };
    }

    Goal::Goal(SimulatedArena& arena, units::meter_t xDimension, units::meter_t yDimension, units::meter_t height, std::string gamePieceType,
               const frc::Translation3d& position, bool isBlue, int max, bool allowGrounded)
        : xyBox_(frc::Pose2d{position.X(), position.Y(), frc::Rotation2d{}}, xDimension, yDimension)
        , height_(height)
        , elevation_(position.Z())
        , gamePieceType_(std::move(gamePieceType))
        , position_(position)
        , arena_(arena)
        , max_(max)
        , isBlue(isBlue)
        , minZMeters_(position.Z().value())
        , maxZMeters_(position.Z().value() + height.value())
        , allowGrounded_(allowGrounded)
        , rotationChecker_(AnyRotation())
        , positionChecker_(Box(xyBox_, minZMeters_, maxZMeters_))
        , velocityValidator_([](const gamepieces::GamePiece&) { return true; }) {}

    Goal::Goal(SimulatedArena& arena, units::meter_t xDimension, units::meter_t yDimension, units::meter_t height, std::string gamePieceType,
               const frc::Translation3d& position, bool isBlue, bool allowsGrounded)
        : Goal(arena, xDimension, yDimension, height, std::move(gamePieceType), position, isBlue, 99999, allowsGrounded) {}

    void Goal::SimulationSubTick([[maybe_unused]] int subTickNum) {
        if (gamePieceCount_ >= max_) return;

        int remainingScorable = max_ - gamePieceCount_;
        for (gamepieces::GamePiece* gamePiece : arena_.GetGamePiecesByType(gamePieceType_)) {
            if (remainingScorable <= 0) break;
            if (!CheckGrounded(*gamePiece) || !CheckValidity(*gamePiece)) continue;
            remainingScorable--;
            gamePieceCount_++;
            AddPoints();
            gamePiece->TriggerHitTargetCallBack();
            arena_.RemovePiece(*gamePiece);
        }
    }

    void Goal::SetNeededAngle(const frc::Rotation3d& angle, units::degree_t angleTolerance) {
        rotationChecker_ = AbsoluteAngle(angle, angleTolerance);
    }

    Goal& Goal::WithCustomRotationChecker(RotationChecker checker) {
        rotationChecker_ = std::move(checker);
        return *this;
    }

    Goal& Goal::WithCustomRotationValidator(RotationChecker validator) {
        rotationChecker_ = std::move(validator);
        return *this;
    }

    Goal& Goal::WithCustomPositionChecker(PositionChecker checker) {
        positionChecker_ = std::move(checker);
        return *this;
    }

    Goal& Goal::WithCustomCollisionPredicate(PositionChecker predicate) {
        positionChecker_ = std::move(predicate);
        return *this;
    }

    Goal& Goal::WithCustomVelocityValidator(VelocityValidator validator) {
        velocityValidator_ = std::move(validator);
        return *this;
    }

    void Goal::Clear() {
        gamePieceCount_ = 0;
    }

    bool Goal::CheckValidity(const gamepieces::GamePiece& gamePiece) const {
        return rotationChecker_(gamePiece) && velocityValidator_(gamePiece) && positionChecker_(gamePiece.GetPose3d().Translation());
    }

    bool Goal::CheckGrounded(const gamepieces::GamePiece& gamePiece) const {
        return allowGrounded_ || !gamePiece.IsGrounded();
    }

    bool Goal::CheckCollision(const gamepieces::GamePiece& gamePiece) const {
        const frc::Pose3d pose = gamePiece.GetPose3d();

        return xyBox_.Contains(pose.Translation().ToTranslation2d()) && pose.Z().value() >= minZMeters_ && pose.Z().value() <= maxZMeters_;
    }
} // namespace maplesim::simulation
