#include "pch.h"

#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"

#include <algorithm>

#include <units/angular_velocity.h>
#include <units/torque.h>

#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"

namespace maplesim::simulation::drivesims::configs {
    SwerveModuleSimulationConfig::SwerveModuleSimulationConfig(frc::DCMotor driveMotorModel, frc::DCMotor steerMotorModel, double driveGearRatio,
                                                               double steerGearRatio, units::volt_t driveFrictionVoltage, units::volt_t steerFrictionVoltage,
                                                               units::meter_t wheelRadius, units::kilogram_square_meter_t steerRotationalInertia,
                                                               double wheelsCoefficientOfFriction)
        : driveMotorConfigs(driveMotorModel, driveGearRatio, units::kilogram_square_meter_t{0.0}, driveFrictionVoltage)
        , steerMotorConfigs(steerMotorModel, steerGearRatio, steerRotationalInertia, steerFrictionVoltage)
        , driveGearRatio(driveGearRatio)
        , steerGearRatio(steerGearRatio)
        , wheelsCoefficientOfFriction(wheelsCoefficientOfFriction)
        , driveFrictionVoltage(driveFrictionVoltage)
        , wheelRadius(wheelRadius) {
        BoundingCheck::Check(driveGearRatio, 4, 24, "drive gear ratio", "times reduction");
        BoundingCheck::Check(steerGearRatio, 6, 50, "steer gear ratio", "times reduction");
        BoundingCheck::Check(driveFrictionVoltage.value(), 0.01, 0.35, "drive friction voltage", "volts");
        BoundingCheck::Check(steerFrictionVoltage.value(), 0.01, 0.6, "steer friction voltage", "volts");
        BoundingCheck::Check(units::inch_t{wheelRadius}.value(), 1, 3.2, "drive wheel radius", "inches");
        BoundingCheck::Check(steerRotationalInertia.value(), 0.005, 0.06, "steer rotation inertia", "kg * m^2");
        BoundingCheck::Check(wheelsCoefficientOfFriction, 0.6, 2.5, "tire coefficient of friction", "");
    }

    std::unique_ptr<SwerveModuleSimulation> SwerveModuleSimulationConfig::operator()() const {
        return std::make_unique<SwerveModuleSimulation>(*this);
    }

    units::newton_t SwerveModuleSimulationConfig::GetGrippingForce(units::newton_t gravityForceOnModule) const {
        return gravityForceOnModule * wheelsCoefficientOfFriction;
    }

    units::meters_per_second_t SwerveModuleSimulationConfig::MaximumGroundSpeed() const {
        return units::meters_per_second_t{driveMotorConfigs.FreeSpinMechanismVelocity().value() * wheelRadius.value()};
    }

    units::newton_t SwerveModuleSimulationConfig::GetTheoreticalPropellingForcePerModule(units::kilogram_t robotMass, int modulesCount,
                                                                                         units::ampere_t statorCurrentLimit) const {
        const double maxThrustNewtons = driveMotorConfigs.CalculateTorque(statorCurrentLimit).value() / wheelRadius.value();
        const double maxGrippingNewtons = 9.8 * robotMass.value() / modulesCount * wheelsCoefficientOfFriction;
        return units::newton_t{std::min(maxThrustNewtons, maxGrippingNewtons)};
    }

    units::meters_per_second_squared_t SwerveModuleSimulationConfig::MaxAcceleration(units::kilogram_t robotMass, int modulesCount,
                                                                                     units::ampere_t statorCurrentLimit) const {
        return GetTheoreticalPropellingForcePerModule(robotMass, modulesCount, statorCurrentLimit) * modulesCount / robotMass;
    }
} // namespace maplesim::simulation::drivesims::configs
