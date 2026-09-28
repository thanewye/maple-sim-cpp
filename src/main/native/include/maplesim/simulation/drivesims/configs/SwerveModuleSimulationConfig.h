#pragma once

#include <memory>

#include <frc/system/plant/DCMotor.h>
#include <units/acceleration.h>
#include <units/current.h>
#include <units/force.h>
#include <units/length.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "maplesim/simulation/motorsims/SimMotorConfigs.h"

namespace maplesim::simulation::drivesims {
    class SwerveModuleSimulation;
} // namespace maplesim::simulation::drivesims

namespace maplesim::simulation::drivesims::configs {
    /** Physical properties of one swerve module; also a factory for SwerveModuleSimulation. */
    class SwerveModuleSimulationConfig {
    public:
        SwerveModuleSimulationConfig(frc::DCMotor driveMotorModel, frc::DCMotor steerMotorModel, double driveGearRatio, double steerGearRatio,
                                     units::volt_t driveFrictionVoltage, units::volt_t steerFrictionVoltage, units::meter_t wheelRadius,
                                     units::kilogram_square_meter_t steerRotationalInertia, double wheelsCoefficientOfFriction);

        [[nodiscard]] std::unique_ptr<SwerveModuleSimulation> operator()() const;

        [[nodiscard]] units::newton_t GetGrippingForce(units::newton_t gravityForceOnModule) const;
        [[nodiscard]] units::meters_per_second_t MaximumGroundSpeed() const;
        [[nodiscard]] units::newton_t GetTheoreticalPropellingForcePerModule(units::kilogram_t robotMass, int modulesCount,
                                                                             units::ampere_t statorCurrentLimit) const;
        [[nodiscard]] units::meters_per_second_squared_t MaxAcceleration(units::kilogram_t robotMass, int modulesCount,
                                                                         units::ampere_t statorCurrentLimit) const;

        motorsims::SimMotorConfigs driveMotorConfigs;
        motorsims::SimMotorConfigs steerMotorConfigs;
        double driveGearRatio;
        double steerGearRatio;
        double wheelsCoefficientOfFriction;
        units::volt_t driveFrictionVoltage;
        units::meter_t wheelRadius;
    };
} // namespace maplesim::simulation::drivesims::configs
