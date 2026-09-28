#include "pch.h"

#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

#include <algorithm>
#include <string>
#include <utility>

#include <frc/system/plant/DCMotor.h>
#include <units/math.h>

#include "maplesim/simulation/drivesims/COTS.h"
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"

namespace maplesim::simulation::drivesims::configs {
    DriveTrainSimulationConfig::DriveTrainSimulationConfig(units::kilogram_t robotMass, units::meter_t bumperLengthX, units::meter_t bumperWidthY,
                                                           units::meter_t trackLengthX, units::meter_t trackWidthY, GyroSimulationFactory gyroSimulationFactory,
                                                           SwerveModuleSimulationFactory swerveModuleSimulationFactory)
        : robotMass(robotMass)
        , bumperLengthX(bumperLengthX)
        , bumperWidthY(bumperWidthY) {
        WithTrackLengthTrackWidth(trackLengthX, trackWidthY);
        WithSwerveModule(std::move(swerveModuleSimulationFactory));
        this->gyroSimulationFactory = std::move(gyroSimulationFactory);
        CheckRobotMass();
        CheckBumperSize();
    }

    DriveTrainSimulationConfig::DriveTrainSimulationConfig(units::kilogram_t robotMass, units::meter_t bumperLengthX, units::meter_t bumperWidthY,
                                                           units::meter_t trackLengthX, units::meter_t trackWidthY, GyroSimulationFactory gyroSimulationFactory,
                                                           std::array<SwerveModuleSimulationFactory, kSwerveModuleCount> swerveModuleSimulationFactories)
        : robotMass(robotMass)
        , bumperLengthX(bumperLengthX)
        , bumperWidthY(bumperWidthY) {
        WithTrackLengthTrackWidth(trackLengthX, trackWidthY);
        WithSwerveModules(std::move(swerveModuleSimulationFactories));
        this->gyroSimulationFactory = std::move(gyroSimulationFactory);
        CheckRobotMass();
        CheckBumperSize();
    }

    DriveTrainSimulationConfig DriveTrainSimulationConfig::Default() {
        return DriveTrainSimulationConfig{units::kilogram_t{45},
                                          units::meter_t{0.76},
                                          units::meter_t{0.76},
                                          units::meter_t{0.52},
                                          units::meter_t{0.52},
                                          COTS::OfPigeon2(),
                                          COTS::OfMark4(frc::DCMotor::Falcon500(1), frc::DCMotor::Falcon500(1), COTS::Wheels::kColsons.cof, 2)};
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithRobotMass(units::kilogram_t robotMass) {
        this->robotMass = robotMass;
        CheckRobotMass();
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithBumperSize(units::meter_t bumperLengthX, units::meter_t bumperWidthY) {
        this->bumperLengthX = bumperLengthX;
        this->bumperWidthY = bumperWidthY;
        CheckBumperSize();
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithTrackLengthTrackWidth(units::meter_t trackLengthX, units::meter_t trackWidthY) {
        BoundingCheck::Check(trackLengthX.value(), 0.2, 1.5, "track length", "meters");
        BoundingCheck::Check(trackWidthY.value(), 0.2, 1.5, "track width", "meters");
        moduleTranslations = {frc::Translation2d{trackLengthX / 2, trackWidthY / 2}, frc::Translation2d{trackLengthX / 2, -trackWidthY / 2},
                              frc::Translation2d{-trackLengthX / 2, trackWidthY / 2}, frc::Translation2d{-trackLengthX / 2, -trackWidthY / 2}};
        return *this;
    }

    DriveTrainSimulationConfig&
    DriveTrainSimulationConfig::WithCustomModuleTranslations(const std::array<frc::Translation2d, kSwerveModuleCount>& moduleTranslations) {
        this->moduleTranslations = moduleTranslations;
        CheckModuleTranslations();
        return *this;
    }

    DriveTrainSimulationConfig&
    DriveTrainSimulationConfig::WithSwerveModules(std::array<SwerveModuleSimulationFactory, kSwerveModuleCount> swerveModuleSimulationFactories) {
        this->swerveModuleSimulationFactories = std::move(swerveModuleSimulationFactories);
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithSwerveModule(SwerveModuleSimulationFactory swerveModuleSimulationFactory) {
        swerveModuleSimulationFactories.fill(swerveModuleSimulationFactory);
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithGyro(GyroSimulationFactory gyroSimulationFactory) {
        this->gyroSimulationFactory = std::move(gyroSimulationFactory);
        return *this;
    }

    physics::kilograms_per_square_meter_t DriveTrainSimulationConfig::GetDensity() const {
        return robotMass / (bumperLengthX * bumperWidthY);
    }

    units::meter_t DriveTrainSimulationConfig::TrackLengthX() const {
        const auto [minModule, maxModule] =
            std::ranges::minmax_element(moduleTranslations, [](const frc::Translation2d& a, const frc::Translation2d& b) { return a.X() < b.X(); });
        return maxModule->X() - minModule->X();
    }

    units::meter_t DriveTrainSimulationConfig::TrackWidthY() const {
        const auto [minModule, maxModule] =
            std::ranges::minmax_element(moduleTranslations, [](const frc::Translation2d& a, const frc::Translation2d& b) { return a.Y() < b.Y(); });
        return maxModule->Y() - minModule->Y();
    }

    units::meter_t DriveTrainSimulationConfig::DriveBaseRadius() const {
        return units::math::hypot(TrackLengthX() / 2, TrackWidthY() / 2);
    }

    void DriveTrainSimulationConfig::CheckRobotMass() const {
        BoundingCheck::Check(robotMass.value(), 10, 80, "robot mass", "kg");
    }

    void DriveTrainSimulationConfig::CheckBumperSize() const {
        BoundingCheck::Check(bumperLengthX.value(), 0.2, 1.5, "bumper length", "meters");
        BoundingCheck::Check(bumperWidthY.value(), 0.2, 1.5, "bumper width", "meters");
    }

    void DriveTrainSimulationConfig::CheckModuleTranslations() const {
        for (int i = 0; i < kSwerveModuleCount; i++)
            BoundingCheck::Check(moduleTranslations[i].Norm().value(), 0.2, 1.2, "module number " + std::to_string(i) + " translation magnitude", "meters");
    }
} // namespace maplesim::simulation::drivesims::configs
