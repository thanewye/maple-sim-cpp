#pragma once

#include <frc/system/plant/DCMotor.h>

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

    /** Throws frc::RuntimeError on an unknown gear ratio level, as do the other module presets. */
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark4n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark5n(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMark5i(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                                  double firstStageRatio);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveXFlipped(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                                         int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveXS(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                                   int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX2(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                                   int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfSwerveX2S(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                                    int pinionSize);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfMAXSwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);
    [[nodiscard]] configs::SwerveModuleSimulationConfig OfThriftySwerve(frc::DCMotor driveMotor, frc::DCMotor steerMotor, double wheelCOF, int gearRatioLevel);

    [[nodiscard]] configs::GyroSimulationFactory OfPigeon2();
    [[nodiscard]] configs::GyroSimulationFactory OfNav2X();
    [[nodiscard]] configs::GyroSimulationFactory OfGenericGyro();
} // namespace maplesim::simulation::drivesims::COTS
