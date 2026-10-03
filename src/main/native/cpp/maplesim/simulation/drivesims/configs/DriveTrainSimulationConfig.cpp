#include "pch.h"

#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"

#include <algorithm>
#include <string>
#include <utility>

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/math.hpp>

#include "maplesim/simulation/drivesims/COTS.h"
#include "maplesim/simulation/drivesims/GyroSimulation.h"
#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"
#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"

namespace maplesim::simulation::drivesims::configs {
    DriveTrainSimulationConfig::DriveTrainSimulationConfig(wpi::units::kilogram_t robotMass, wpi::units::meter_t bumperLengthX,
                                                           wpi::units::meter_t bumperWidthY, wpi::units::meter_t trackLengthX, wpi::units::meter_t trackWidthY,
                                                           GyroSimulationFactory gyroSimulationFactory,
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

    DriveTrainSimulationConfig::DriveTrainSimulationConfig(wpi::units::kilogram_t robotMass, wpi::units::meter_t bumperLengthX,
                                                           wpi::units::meter_t bumperWidthY, wpi::units::meter_t trackLengthX, wpi::units::meter_t trackWidthY,
                                                           GyroSimulationFactory gyroSimulationFactory,
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
        return DriveTrainSimulationConfig{wpi::units::kilogram_t{45},
                                          wpi::units::meter_t{0.76},
                                          wpi::units::meter_t{0.76},
                                          wpi::units::meter_t{0.52},
                                          wpi::units::meter_t{0.52},
                                          COTS::OfPigeon2(),
                                          COTS::OfMark4(wpi::math::DCMotor::Falcon500(1), wpi::math::DCMotor::Falcon500(1), COTS::Wheels::kColsons.cof, 2)};
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithRobotMass(wpi::units::kilogram_t robotMass) {
        this->robotMass = robotMass;
        CheckRobotMass();
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithBumperSize(wpi::units::meter_t bumperLengthX, wpi::units::meter_t bumperWidthY) {
        this->bumperLengthX = bumperLengthX;
        this->bumperWidthY = bumperWidthY;
        CheckBumperSize();
        return *this;
    }

    DriveTrainSimulationConfig& DriveTrainSimulationConfig::WithTrackLengthTrackWidth(wpi::units::meter_t trackLengthX, wpi::units::meter_t trackWidthY) {
        BoundingCheck::Check(trackLengthX.value(), 0.2, 1.5, "track length", "meters");
        BoundingCheck::Check(trackWidthY.value(), 0.2, 1.5, "track width", "meters");
        moduleTranslations = {wpi::math::Translation2d{trackLengthX / 2, trackWidthY / 2}, wpi::math::Translation2d{trackLengthX / 2, -trackWidthY / 2},
                              wpi::math::Translation2d{-trackLengthX / 2, trackWidthY / 2}, wpi::math::Translation2d{-trackLengthX / 2, -trackWidthY / 2}};
        return *this;
    }

    DriveTrainSimulationConfig&
    DriveTrainSimulationConfig::WithCustomModuleTranslations(const std::array<wpi::math::Translation2d, kSwerveModuleCount>& moduleTranslations) {
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

    wpi::units::meter_t DriveTrainSimulationConfig::TrackLengthX() const {
        const auto [minModule, maxModule] =
            std::ranges::minmax_element(moduleTranslations, [](const wpi::math::Translation2d& a, const wpi::math::Translation2d& b) { return a.X() < b.X(); });
        return maxModule->X() - minModule->X();
    }

    wpi::units::meter_t DriveTrainSimulationConfig::TrackWidthY() const {
        const auto [minModule, maxModule] =
            std::ranges::minmax_element(moduleTranslations, [](const wpi::math::Translation2d& a, const wpi::math::Translation2d& b) { return a.Y() < b.Y(); });
        return maxModule->Y() - minModule->Y();
    }

    wpi::units::meter_t DriveTrainSimulationConfig::DriveBaseRadius() const {
        return wpi::units::math::hypot(TrackLengthX() / 2, TrackWidthY() / 2);
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
