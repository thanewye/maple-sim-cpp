#include "pch.h"

#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

#include <utility>

#include <wpi/math/geometry/Rotation3d.hpp>

#include "maplesim/physics/Fixture.h"

namespace maplesim::simulation::gamepieces {
    GamePieceOnFieldSimulation::GamePieceOnFieldSimulation(const GamePieceInfo& info, const wpi::math::Pose2d& initialPose)
        : GamePieceOnFieldSimulation(
              info, [gamePieceHeight = info.gamePieceHeight] { return gamePieceHeight.value() / 2; }, initialPose, wpi::math::Translation2d{}) {}

    GamePieceOnFieldSimulation::GamePieceOnFieldSimulation(const GamePieceInfo& info, std::function<double()> zPositionSupplier,
                                                           const wpi::math::Pose2d& initialPose, const wpi::math::Translation2d& initialVelocityMPS)
        : type(info.type)
        , zPositionSupplier_(std::move(zPositionSupplier)) {
        physics::FixtureMaterial material;
        material.friction = kCoefficientOfFriction;
        material.restitution = info.coefficientOfRestitution;
        material.restitutionThreshold = wpi::units::meters_per_second_t{kMinimumBouncingVelocity};
        material.density = info.gamePieceMass / info.shape.GetArea();
        AddFixture(info.shape, material);
        SetBodyType(physics::BodyType::kDynamic);

        SetLinearDamping(info.linearDamping);
        SetAngularDamping(info.angularDamping);
        SetBullet(true);

        SetPose(initialPose);
        SetLinearVelocity(physics::LinearVelocity2d{wpi::units::meters_per_second_t{initialVelocityMPS.X().value()},
                                                    wpi::units::meters_per_second_t{initialVelocityMPS.Y().value()}});
    }

    wpi::math::Pose3d GamePieceOnFieldSimulation::GetPose3d() const {
        const wpi::math::Pose2d pose2d = GetPoseOnField();
        return wpi::math::Pose3d{pose2d.X(), pose2d.Y(), wpi::units::meter_t{zPositionSupplier_()},
                                 wpi::math::Rotation3d{wpi::units::radian_t{0}, wpi::units::radian_t{0}, pose2d.Rotation().Radians()}};
    }

    wpi::math::Translation3d GamePieceOnFieldSimulation::GetVelocity3dMPS() const {
        const physics::LinearVelocity2d linearVelocity = GetLinearVelocity();
        return wpi::math::Translation3d{wpi::units::meter_t{linearVelocity.x.value()}, wpi::units::meter_t{linearVelocity.y.value()}, wpi::units::meter_t{0}};
    }
} // namespace maplesim::simulation::gamepieces
