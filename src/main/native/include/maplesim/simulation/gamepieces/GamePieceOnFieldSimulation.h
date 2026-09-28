#pragma once

#include <functional>
#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/length.h>
#include <units/mass.h>

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
            units::meter_t gamePieceHeight;
            units::kilogram_t gamePieceMass;
            double linearDamping;
            double angularDamping;
            double coefficientOfRestitution;
        };

        GamePieceOnFieldSimulation(const GamePieceInfo& info, const frc::Pose2d& initialPose);
        GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<double()> zPositionSupplier, const frc::Pose2d& initialPose,
                                   const frc::Translation2d& initialVelocityMPS);

        [[nodiscard]] frc::Pose2d GetPoseOnField() const { return GetPose(); }
        [[nodiscard]] frc::Pose3d GetPose3d() const override;

        virtual void OnIntake([[maybe_unused]] const std::string& intakeTargetGamePieceType) {}

        [[nodiscard]] const std::string& GetType() const override { return type; }
        [[nodiscard]] frc::Translation3d GetVelocity3dMPS() const override;
        [[nodiscard]] bool IsGrounded() const override { return true; }
        void TriggerHitTargetCallBack() override {}

        const std::string type;

    private:
        std::function<double()> zPositionSupplier_;
    };
} // namespace maplesim::simulation::gamepieces
