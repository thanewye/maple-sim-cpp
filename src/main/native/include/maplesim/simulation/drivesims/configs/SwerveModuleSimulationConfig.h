#pragma once

#include <memory>

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include "maplesim/simulation/motorsims/SimMotorConfigs.h"

namespace maplesim::simulation::drivesims {
    class SwerveModuleSimulation;
} // namespace maplesim::simulation::drivesims

namespace maplesim::simulation::drivesims::configs {
    /** Physical properties of one swerve module; also a factory for SwerveModuleSimulation. */
    class SwerveModuleSimulationConfig {
    public:
        SwerveModuleSimulationConfig(wpi::math::DCMotor driveMotorModel, wpi::math::DCMotor steerMotorModel, double driveGearRatio, double steerGearRatio,
                                     wpi::units::volt_t driveFrictionVoltage, wpi::units::volt_t steerFrictionVoltage, wpi::units::meter_t wheelRadius,
                                     wpi::units::kilogram_square_meter_t steerRotationalInertia, double wheelsCoefficientOfFriction);

        [[nodiscard]] std::unique_ptr<SwerveModuleSimulation> operator()() const;

        [[nodiscard]] wpi::units::newton_t GetGrippingForce(wpi::units::newton_t gravityForceOnModule) const;
        [[nodiscard]] wpi::units::meters_per_second_t MaximumGroundSpeed() const;
        [[nodiscard]] wpi::units::newton_t GetTheoreticalPropellingForcePerModule(wpi::units::kilogram_t robotMass, int modulesCount,
                                                                                  wpi::units::ampere_t statorCurrentLimit) const;
        [[nodiscard]] wpi::units::meters_per_second_squared_t MaxAcceleration(wpi::units::kilogram_t robotMass, int modulesCount,
                                                                              wpi::units::ampere_t statorCurrentLimit) const;

        motorsims::SimMotorConfigs driveMotorConfigs;
        motorsims::SimMotorConfigs steerMotorConfigs;
        double driveGearRatio;
        double steerGearRatio;
        double wheelsCoefficientOfFriction;
        wpi::units::volt_t driveFrictionVoltage;
        wpi::units::meter_t wheelRadius;
    };
} // namespace maplesim::simulation::drivesims::configs
