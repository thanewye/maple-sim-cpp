#pragma once

#include <array>
#include <functional>
#include <memory>

#include <frc/geometry/Translation2d.h>
#include <units/length.h>
#include <units/mass.h>

#include "maplesim/physics/Fixture.h"

namespace maplesim::simulation::drivesims {
    class GyroSimulation;
    class SwerveModuleSimulation;
} // namespace maplesim::simulation::drivesims

namespace maplesim::simulation::drivesims::configs {
    inline constexpr int kSwerveModuleCount = 4;

    using SwerveModuleSimulationFactory = std::function<std::unique_ptr<SwerveModuleSimulation>()>;
    using GyroSimulationFactory = std::function<std::unique_ptr<GyroSimulation>()>;

    /** Mass, bumper, module layout, module factories and gyro factory of a simulated swerve drivetrain; modules are ordered FL, FR, BL, BR. */
    class DriveTrainSimulationConfig {
    public:
        DriveTrainSimulationConfig(units::kilogram_t robotMass, units::meter_t bumperLengthX, units::meter_t bumperWidthY, units::meter_t trackLengthX,
                                   units::meter_t trackWidthY, GyroSimulationFactory gyroSimulationFactory,
                                   SwerveModuleSimulationFactory swerveModuleSimulationFactory);
        DriveTrainSimulationConfig(units::kilogram_t robotMass, units::meter_t bumperLengthX, units::meter_t bumperWidthY, units::meter_t trackLengthX,
                                   units::meter_t trackWidthY, GyroSimulationFactory gyroSimulationFactory,
                                   std::array<SwerveModuleSimulationFactory, kSwerveModuleCount> swerveModuleSimulationFactories);

        /** 45 kg robot with 0.76 m bumpers and Falcon-driven L2 Mark4 modules on a 0.52 m square track. */
        [[nodiscard]] static DriveTrainSimulationConfig Default();

        DriveTrainSimulationConfig& WithRobotMass(units::kilogram_t robotMass);
        DriveTrainSimulationConfig& WithBumperSize(units::meter_t bumperLengthX, units::meter_t bumperWidthY);
        DriveTrainSimulationConfig& WithTrackLengthTrackWidth(units::meter_t trackLengthX, units::meter_t trackWidthY);
        DriveTrainSimulationConfig& WithCustomModuleTranslations(const std::array<frc::Translation2d, kSwerveModuleCount>& moduleTranslations);
        DriveTrainSimulationConfig& WithSwerveModules(std::array<SwerveModuleSimulationFactory, kSwerveModuleCount> swerveModuleSimulationFactories);
        DriveTrainSimulationConfig& WithSwerveModule(SwerveModuleSimulationFactory swerveModuleSimulationFactory);
        DriveTrainSimulationConfig& WithGyro(GyroSimulationFactory gyroSimulationFactory);

        [[nodiscard]] physics::kilograms_per_square_meter_t GetDensity() const;
        [[nodiscard]] units::meter_t TrackLengthX() const;
        [[nodiscard]] units::meter_t TrackWidthY() const;
        [[nodiscard]] units::meter_t DriveBaseRadius() const;

        units::kilogram_t robotMass;
        units::meter_t bumperLengthX;
        units::meter_t bumperWidthY;
        std::array<SwerveModuleSimulationFactory, kSwerveModuleCount> swerveModuleSimulationFactories;
        GyroSimulationFactory gyroSimulationFactory;
        std::array<frc::Translation2d, kSwerveModuleCount> moduleTranslations;

    private:
        void CheckRobotMass() const;
        void CheckBumperSize() const;
        void CheckModuleTranslations() const;
    };
} // namespace maplesim::simulation::drivesims::configs
