#include "pch.h"

#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

#include <utility>

#include <frc/geometry/Rotation3d.h>

#include "maplesim/physics/Fixture.h"

namespace maplesim::simulation::gamepieces {
    GamePieceOnFieldSimulation::GamePieceOnFieldSimulation(const GamePieceInfo& info, const frc::Pose2d& initialPose)
        : GamePieceOnFieldSimulation(
              info, [gamePieceHeight = info.gamePieceHeight] { return gamePieceHeight.value() / 2; }, initialPose, frc::Translation2d{}) {}

    GamePieceOnFieldSimulation::GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<double()> zPositionSupplier, const frc::Pose2d& initialPose,
                                                           const frc::Translation2d& initialVelocityMPS)
        : type(info.type)
        , zPositionSupplier_(std::move(zPositionSupplier)) {
        physics::FixtureMaterial material;
        material.friction = kCoefficientOfFriction;
        material.restitution = info.coefficientOfRestitution;
        material.restitutionThreshold = units::meters_per_second_t{kMinimumBouncingVelocity};
        material.density = info.gamePieceMass / info.shape.GetArea();
        AddFixture(info.shape, material);
        SetBodyType(physics::BodyType::kDynamic);

        SetLinearDamping(info.linearDamping);
        SetAngularDamping(info.angularDamping);
        SetBullet(true);

        SetPose(initialPose);
        SetLinearVelocity(
            physics::LinearVelocity2d{units::meters_per_second_t{initialVelocityMPS.X().value()}, units::meters_per_second_t{initialVelocityMPS.Y().value()}});
    }

    frc::Pose3d GamePieceOnFieldSimulation::GetPose3d() const {
        const frc::Pose2d pose2d = GetPoseOnField();
        return frc::Pose3d{pose2d.X(), pose2d.Y(), units::meter_t{zPositionSupplier_()},
                           frc::Rotation3d{units::radian_t{0}, units::radian_t{0}, pose2d.Rotation().Radians()}};
    }

    frc::Translation3d GamePieceOnFieldSimulation::GetVelocity3dMPS() const {
        const physics::LinearVelocity2d linearVelocity = GetLinearVelocity();
        return frc::Translation3d{units::meter_t{linearVelocity.x.value()}, units::meter_t{linearVelocity.y.value()}, units::meter_t{0}};
    }
} // namespace maplesim::simulation::gamepieces
