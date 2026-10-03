#pragma once

#include <wpi/math/system/DCMotor.hpp>

#include "maplesim/simulation/drivesims/configs/DriveTrainSimulationConfig.h"
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"

namespace maplesim::simulation::drivesims::COTS {
    struct Wheel {
        double cof;
    };

    namespace Wheels {
        inline constexpr Wheel kColsons{0.899};
        inline constexpr Wheel kDefaultNeopreneTread{1.426};
        inline constexpr Wheel kBlueNitrileTread{1.542};
        inline constexpr Wheel kVexGripV2{1.916};
        inline constexpr Wheel kSlsPrintedWheels{2.106};
    } // namespace Wheels

    /** Throws wpi::RuntimeError on an unknown gear ratio level, as do the other module presets. */
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4i(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                 int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4n(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                 int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark5n(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                 int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark5i(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                 int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                  int gearRatioLevel, double firstStageRatio);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveXFlipped(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                         int gearRatioLevel, int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveXS(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                   int gearRatioLevel, int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX2(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                   int gearRatioLevel, int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX2S(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                    int gearRatioLevel, int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMAXSwerve(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                    int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfThriftySwerve(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF,
                                                                        int gearRatioLevel);

    [[nodiscard]] configs::GyroSimulationFactory OfPigeon2();
    [[nodiscard]] configs::GyroSimulationFactory OfNav2X();
    [[nodiscard]] configs::GyroSimulationFactory OfGenericGyro();
} // namespace maplesim::simulation::drivesims::COTS
