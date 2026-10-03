#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>

#include "maplesim/physics/Body.h"
#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

namespace maplesim::simulation::drivesims {
    /** Rigid chassis body with the bumpers as its collision shape; subclasses apply drive forces every sub-tick. */
    class AbstractDriveTrainSimulation : public physics::Body {
    public:
        static constexpr double kBumperCoefficientOfFriction = 0.65;
        static constexpr double kBumperCoefficientOfRestitution = 0.08;

        /** Teleports the chassis and stops its linear motion; angular velocity is kept. */
        void SetSimulationWorldPose(const wpi::math::Pose2d& robotPose);
        /** Sets the field-relative chassis velocity. */
        void SetRobotSpeeds(const wpi::math::ChassisVelocities& givenSpeeds);

        virtual void SimulationSubTick() = 0;

        [[nodiscard]] wpi::math::Pose2d GetSimulatedDriveTrainPose() const { return GetPose(); }
        [[nodiscard]] wpi::math::ChassisVelocities GetDriveTrainSimulatedChassisSpeedsRobotRelative() const;
        [[nodiscard]] wpi::math::ChassisVelocities GetDriveTrainSimulatedChassisSpeedsFieldRelative() const { return GetVelocity(); }

        const configs::DriveTrainSimulationConfig config;

    protected:
        AbstractDriveTrainSimulation(configs::DriveTrainSimulationConfig config, const wpi::math::Pose2d& initialPoseOnField);
    };
} // namespace maplesim::simulation::drivesims
