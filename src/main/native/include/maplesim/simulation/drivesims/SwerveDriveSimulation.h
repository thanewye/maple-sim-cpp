#pragma once

#include <array>
#include <memory>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/force.h>
#include <units/length.h>
#include <units/velocity.h>

#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

namespace maplesim::simulation::drivesims {
    /** Swerve chassis driven by four module sims, with tire friction resisting motion the modules do not command. */
    class SwerveDriveSimulation : public AbstractDriveTrainSimulation {
    public:
        using SwerveModuleSimulations = std::array<std::unique_ptr<SwerveModuleSimulation>, configs::kSwerveModuleCount>;

        SwerveDriveSimulation(configs::DriveTrainSimulationConfig config, const frc::Pose2d& initialPoseOnField);

        void SimulationSubTick() override;

        [[nodiscard]] units::meters_per_second_t MaxLinearVelocity() const;
        [[nodiscard]] units::meters_per_second_squared_t MaxLinearAcceleration(units::ampere_t statorCurrentLimit) const;
        [[nodiscard]] units::meter_t DriveBaseRadius() const { return config.DriveBaseRadius(); }
        [[nodiscard]] units::radians_per_second_t MaxAngularVelocity() const;
        [[nodiscard]] units::radians_per_second_squared_t MaxAngularAcceleration(units::ampere_t statorCurrentLimit) const;

        [[nodiscard]] const SwerveModuleSimulations& GetModules() const { return moduleSimulations_; }
        [[nodiscard]] GyroSimulation& GetGyroSimulation() { return *gyroSimulation_; }

    protected:
        virtual void SimulateChassisFrictionForce();
        virtual void SimulateChassisFrictionTorque();
        virtual void SimulateModulePropellingForces();

        [[nodiscard]] frc::ChassisSpeeds GetDesiredSpeed() const;
        [[nodiscard]] frc::ChassisSpeeds GetModuleSpeeds() const;

        const std::array<frc::Translation2d, configs::kSwerveModuleCount> moduleTranslations_;
        const std::unique_ptr<GyroSimulation> gyroSimulation_;
        const frc::SwerveDriveKinematics<configs::kSwerveModuleCount> kinematics_;

    private:
        SwerveModuleSimulations moduleSimulations_;
        const units::newton_t gravityForceOnEachModule_;
        frc::Translation2d previousModuleSpeedsFieldRelative_;
    };
} // namespace maplesim::simulation::drivesims
