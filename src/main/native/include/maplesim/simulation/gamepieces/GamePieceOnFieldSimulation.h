#pragma once

#include <functional>
#include <string>

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>

#include "maplesim/physics/Body.h"
#include "maplesim/physics/Shape.h"
#include "maplesim/simulation/gamepieces/GamePiece.h"

namespace maplesim::simulation::gamepieces {
    /** A game piece resting on the field as a dynamic body that robots can push and intake. */
    class GamePieceOnFieldSimulation : public physics::Body, public GamePiece {
    public:
        static constexpr double kCoefficientOfFriction = 0.8;
        static constexpr double kMinimumBouncingVelocity = 0.2;

        struct GamePieceInfo {
            std::string type;
            physics::Shape shape;
            wpi::units::meter_t gamePieceHeight;
            wpi::units::kilogram_t gamePieceMass;
            double linearDamping;
            double angularDamping;
            double coefficientOfRestitution;
        };

        GamePieceOnFieldSimulation(const GamePieceInfo& info, const wpi::math::Pose2d& initialPose);
        GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<double()> zPositionSupplier, const wpi::math::Pose2d& initialPose,
                                   const wpi::math::Translation2d& initialVelocityMPS);

        [[nodiscard]] wpi::math::Pose2d GetPoseOnField() const { return GetPose(); }
        [[nodiscard]] wpi::math::Pose3d GetPose3d() const override;

        virtual void OnIntake([[maybe_unused]] const std::string& intakeTargetGamePieceType) {}

        [[nodiscard]] const std::string& GetType() const override { return type; }
        [[nodiscard]] wpi::math::Translation3d GetVelocity3dMPS() const override;
        [[nodiscard]] bool IsGrounded() const override { return true; }
        void TriggerHitTargetCallBack() override {}

        const std::string type;

    private:
        std::function<double()> zPositionSupplier_;
    };
} // namespace maplesim::simulation::gamepieces
