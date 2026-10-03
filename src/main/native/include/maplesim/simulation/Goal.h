#pragma once

#include <functional>
#include <string>
#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/math/shape/Rectangle2d.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>

#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/simulation/gamepieces/GamePiece.h"

namespace maplesim::simulation {
    /** A scoring zone that consumes matching game pieces entering its volume and awards points for them. */
    class Goal : public SimulatedArena::Simulatable {
    public:
        using PositionChecker = std::function<bool(const wpi::math::Translation3d& position)>;
        using RotationChecker = std::function<bool(const gamepieces::GamePiece& gamePiece)>;
        using VelocityValidator = std::function<bool(const gamepieces::GamePiece& gamePiece)>;

        [[nodiscard]] static wpi::math::Rotation3d FlipRotation(const wpi::math::Rotation3d& toFlip);
        [[nodiscard]] static PositionChecker Box(const wpi::math::Rectangle2d& xyBox, double minZMeters, double maxZMeters);
        [[nodiscard]] static RotationChecker AbsoluteAngle(const wpi::math::Rotation3d& expectedAngle, wpi::units::degree_t tolerance);
        [[nodiscard]] static RotationChecker PitchOnly(double expectedPitchRadians, wpi::units::radian_t tolerance);
        [[nodiscard]] static RotationChecker AnyRotation();

        Goal(SimulatedArena& arena, wpi::units::meter_t xDimension, wpi::units::meter_t yDimension, wpi::units::meter_t height, std::string gamePieceType,
             const wpi::math::Translation3d& position, bool isBlue, int max, bool allowGrounded);
        Goal(SimulatedArena& arena, wpi::units::meter_t xDimension, wpi::units::meter_t yDimension, wpi::units::meter_t height, std::string gamePieceType,
             const wpi::math::Translation3d& position, bool isBlue, bool allowsGrounded);

        void SimulationSubTick(int subTickNum) override;

        void SetNeededAngle(const wpi::math::Rotation3d& angle, wpi::units::degree_t angleTolerance = wpi::units::degree_t{10});
        Goal& WithCustomRotationChecker(RotationChecker checker);
        Goal& WithCustomRotationValidator(RotationChecker validator);
        Goal& WithCustomPositionChecker(PositionChecker checker);
        Goal& WithCustomCollisionPredicate(PositionChecker predicate);
        Goal& WithCustomVelocityValidator(VelocityValidator validator);

        void Clear();
        [[nodiscard]] bool IsFull() const { return gamePieceCount_ == max_; }
        virtual void Draw(std::vector<wpi::math::Pose3d>& drawList) const = 0;
        [[nodiscard]] int GetGamePieceCount() const { return gamePieceCount_; }

    protected:
        [[nodiscard]] bool CheckValidity(const gamepieces::GamePiece& gamePiece) const;
        [[nodiscard]] bool CheckGrounded(const gamepieces::GamePiece& gamePiece) const;
        /** Unused by the scoring path, which goes through the position checker; kept for subclasses that call it. */
        [[nodiscard]] virtual bool CheckCollision(const gamepieces::GamePiece& gamePiece) const;
        virtual void AddPoints() = 0;

        wpi::math::Rectangle2d xyBox_;
        const wpi::units::meter_t height_;
        const wpi::units::meter_t elevation_;

        const std::string gamePieceType_;
        const wpi::math::Translation3d position_;
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
