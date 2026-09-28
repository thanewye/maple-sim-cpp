#pragma once

#include <functional>
#include <string>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rectangle2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <units/angle.h>
#include <units/length.h>

#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/simulation/gamepieces/GamePiece.h"

namespace maplesim::simulation {
    /** A scoring zone that consumes matching game pieces entering its volume and awards points for them. */
    class Goal : public SimulatedArena::Simulatable {
    public:
        using PositionChecker = std::function<bool(const frc::Translation3d& position)>;
        using RotationChecker = std::function<bool(const gamepieces::GamePiece& gamePiece)>;
        using VelocityValidator = std::function<bool(const gamepieces::GamePiece& gamePiece)>;

        [[nodiscard]] static frc::Rotation3d FlipRotation(const frc::Rotation3d& toFlip);
        [[nodiscard]] static PositionChecker Box(const frc::Rectangle2d& xyBox, double minZMeters, double maxZMeters);
        [[nodiscard]] static RotationChecker AbsoluteAngle(const frc::Rotation3d& expectedAngle, units::degree_t tolerance);
        [[nodiscard]] static RotationChecker PitchOnly(double expectedPitchRadians, units::radian_t tolerance);
        [[nodiscard]] static RotationChecker AnyRotation();

        Goal(SimulatedArena& arena, units::meter_t xDimension, units::meter_t yDimension, units::meter_t height, std::string gamePieceType,
             const frc::Translation3d& position, bool isBlue, int max, bool allowGrounded);
        Goal(SimulatedArena& arena, units::meter_t xDimension, units::meter_t yDimension, units::meter_t height, std::string gamePieceType,
             const frc::Translation3d& position, bool isBlue, bool allowsGrounded);

        void SimulationSubTick(int subTickNum) override;

        void SetNeededAngle(const frc::Rotation3d& angle, units::degree_t angleTolerance = units::degree_t{10});
        Goal& WithCustomRotationChecker(RotationChecker checker);
        Goal& WithCustomRotationValidator(RotationChecker validator);
        Goal& WithCustomPositionChecker(PositionChecker checker);
        Goal& WithCustomCollisionPredicate(PositionChecker predicate);
        Goal& WithCustomVelocityValidator(VelocityValidator validator);

        void Clear();
        [[nodiscard]] bool IsFull() const { return gamePieceCount_ == max_; }
        virtual void Draw(std::vector<frc::Pose3d>& drawList) const = 0;
        [[nodiscard]] int GetGamePieceCount() const { return gamePieceCount_; }

    protected:
        [[nodiscard]] bool CheckValidity(const gamepieces::GamePiece& gamePiece) const;
        [[nodiscard]] bool CheckGrounded(const gamepieces::GamePiece& gamePiece) const;
        /** Unused by the scoring path, which goes through the position checker; kept for subclasses that call it. */
        [[nodiscard]] virtual bool CheckCollision(const gamepieces::GamePiece& gamePiece) const;
        virtual void AddPoints() = 0;

        frc::Rectangle2d xyBox_;
        const units::meter_t height_;
        const units::meter_t elevation_;

        const std::string gamePieceType_;
        const frc::Translation3d position_;
        SimulatedArena& arena_;
        const int max_;

    public:
        const bool isBlue;

    protected:
        int gamePieceCount_ = 0;

        double minZMeters_;
        double maxZMeters_;

        const bool allowGrounded_;

        RotationChecker rotationChecker_;
        PositionChecker positionChecker_;
        VelocityValidator velocityValidator_;
    };
} // namespace maplesim::simulation
