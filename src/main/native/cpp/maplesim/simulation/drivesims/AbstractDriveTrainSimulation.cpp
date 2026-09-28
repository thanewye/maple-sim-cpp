#include "pch.h"

#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"

#include <utility>

#include "maplesim/physics/Fixture.h"
#include "maplesim/physics/Shape.h"

namespace maplesim::simulation::drivesims {
    AbstractDriveTrainSimulation::AbstractDriveTrainSimulation(configs::DriveTrainSimulationConfig config, const frc::Pose2d& initialPoseOnField)
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

    void AbstractDriveTrainSimulation::SetSimulationWorldPose(const frc::Pose2d& robotPose) {
        SetPose(robotPose);
        SetLinearVelocity(physics::LinearVelocity2d{});
    }

    void AbstractDriveTrainSimulation::SetRobotSpeeds(const frc::ChassisSpeeds& givenSpeeds) {
        SetVelocity(givenSpeeds);
    }

    frc::ChassisSpeeds AbstractDriveTrainSimulation::GetDriveTrainSimulatedChassisSpeedsRobotRelative() const {
        return frc::ChassisSpeeds::FromFieldRelativeSpeeds(GetDriveTrainSimulatedChassisSpeedsFieldRelative(), GetSimulatedDriveTrainPose().Rotation());
    }
} // namespace maplesim::simulation::drivesims
