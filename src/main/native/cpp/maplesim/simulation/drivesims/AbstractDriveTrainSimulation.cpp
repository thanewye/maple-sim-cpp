#include "pch.h"

#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"

#include <utility>

#include "maplesim/physics/Fixture.h"
#include "maplesim/physics/Shape.h"

namespace maplesim::simulation::drivesims {
    AbstractDriveTrainSimulation::AbstractDriveTrainSimulation(configs::DriveTrainSimulationConfig config, const wpi::math::Pose2d& initialPoseOnField)
        : config(std::move(config)) {
        physics::FixtureMaterial bumperMaterial;
        bumperMaterial.friction = kBumperCoefficientOfFriction;
        bumperMaterial.restitution = kBumperCoefficientOfRestitution;
        bumperMaterial.density = this->config.GetDensity();
        AddFixture(physics::Shape::Rectangle(this->config.bumperLengthX, this->config.bumperWidthY), bumperMaterial);
        SetBodyType(physics::BodyType::kDynamic);
        SetLinearDamping(0.1);
        SetAngularDamping(0.1);
        SetSimulationWorldPose(initialPoseOnField);
    }

    void AbstractDriveTrainSimulation::SetSimulationWorldPose(const wpi::math::Pose2d& robotPose) {
        SetPose(robotPose);
        SetLinearVelocity(physics::LinearVelocity2d{});
    }

    void AbstractDriveTrainSimulation::SetRobotSpeeds(const wpi::math::ChassisVelocities& givenSpeeds) {
        SetVelocity(givenSpeeds);
    }

    wpi::math::ChassisVelocities AbstractDriveTrainSimulation::GetDriveTrainSimulatedChassisSpeedsRobotRelative() const {
        return GetDriveTrainSimulatedChassisSpeedsFieldRelative().ToRobotRelative(GetSimulatedDriveTrainPose().Rotation());
    }
} // namespace maplesim::simulation::drivesims
