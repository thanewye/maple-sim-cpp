#pragma once

#include <array>
#include <memory>

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

namespace maplesim::simulation::drivesims {
    /** Swerve chassis driven by four module sims, with tire friction resisting motion the modules do not command. */
    class SwerveDriveSimulation : public AbstractDriveTrainSimulation {
    public:
        using SwerveModuleSimulations = std::array<std::unique_ptr<SwerveModuleSimulation>, configs::kSwerveModuleCount>;

        SwerveDriveSimulation(configs::DriveTrainSimulationConfig config, const wpi::math::Pose2d& initialPoseOnField);

        void SimulationSubTick() override;

        [[nodiscard]] wpi::units::meters_per_second_t MaxLinearVelocity() const;
        [[nodiscard]] wpi::units::meters_per_second_squared_t MaxLinearAcceleration(wpi::units::ampere_t statorCurrentLimit) const;
        [[nodiscard]] wpi::units::meter_t DriveBaseRadius() const { return config.DriveBaseRadius(); }
        [[nodiscard]] wpi::units::radians_per_second_t MaxAngularVelocity() const;
        [[nodiscard]] wpi::units::radians_per_second_squared_t MaxAngularAcceleration(wpi::units::ampere_t statorCurrentLimit) const;

        [[nodiscard]] const SwerveModuleSimulations& GetModules() const { return moduleSimulations_; }
        [[nodiscard]] GyroSimulation& GetGyroSimulation() { return *gyroSimulation_; }

    protected:
        virtual void SimulateChassisFrictionForce();
        virtual void SimulateChassisFrictionTorque();
        virtual void SimulateModulePropellingForces();

        [[nodiscard]] wpi::math::ChassisVelocities GetDesiredSpeed() const;
        [[nodiscard]] wpi::math::ChassisVelocities GetModuleSpeeds() const;

        const std::array<wpi::math::Translation2d, configs::kSwerveModuleCount> moduleTranslations_;
        const std::unique_ptr<GyroSimulation> gyroSimulation_;
        const wpi::math::SwerveDriveKinematics<configs::kSwerveModuleCount> kinematics_;

    private:
        SwerveModuleSimulations moduleSimulations_;
        const wpi::units::newton_t gravityForceOnEachModule_;
        wpi::math::Translation2d previousModuleSpeedsFieldRelative_;
    };
} // namespace maplesim::simulation::drivesims
