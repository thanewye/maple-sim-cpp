#include "pch.h"

#include "maplesim/simulation/drivesims/COTS.h"

#include <memory>

#include <wpi/system/Errors.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/voltage.hpp>

#include "maplesim/simulation/drivesims/GyroSimulation.h"

namespace maplesim::simulation::drivesims::COTS {
    namespace {
        [[noreturn]] void ThrowUnknownGearingLevel(int gearRatioLevel) {
            throw WPILIB_MakeError(wpi::err::Error, "Unknown gearing level: {}", gearRatioLevel);
        }

        [[noreturn]] void ThrowUnknownPinionSize(int pinionSize) {
            throw WPILIB_MakeError(wpi::err::Error, "Unknown pinion size: {}", pinionSize);
        }

        [[nodiscard]] configs::SwerveModuleSimulationConfig MakeTypicalModuleConfig(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor,
                                                                                    double driveGearRatio, double steerGearRatio, double wheelCOF) {
            return configs::SwerveModuleSimulationConfig{driveMotor,
                                                         steerMotor,
                                                         driveGearRatio,
                                                         steerGearRatio,
                                                         wpi::units::volt_t{0.1},
                                                         wpi::units::volt_t{0.2},
                                                         wpi::units::inch_t{2},
                                                         wpi::units::kilogram_square_meter_t{0.03},
                                                         wheelCOF};
        }

        [[nodiscard]] double Mark4DriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 8.14;
            case 2:
                return 6.75;
            case 3:
                return 6.12;
            case 4:
                return 5.14;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double Mark4iDriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 8.14;
            case 2:
                return 6.75;
            case 3:
                return 6.12;
            case 4:
                return 5.15;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double Mark4nDriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 7.13;
            case 2:
                return 5.9;
            case 3:
                return 5.36;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double Mark5DriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 7.03;
            case 2:
                return 6.03;
            case 3:
                return 5.27;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double SwerveXSecondStageRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 26.0 / 20.0;
            case 2:
            case 3:
                return 28.0 / 18.0;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double SwerveXFlippedDriveGearRatio(int gearRatioLevel, int pinionSize) {
            switch (gearRatioLevel) {
            case 1:
                switch (pinionSize) {
                case 10:
                    return 8.1;
                case 11:
                    return 7.36;
                case 12:
                    return 6.75;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 2:
                switch (pinionSize) {
                case 10:
                    return 6.72;
                case 11:
                    return 6.11;
                case 12:
                    return 5.6;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 3:
                switch (pinionSize) {
                case 10:
                    return 5.51;
                case 11:
                    return 5.01;
                case 12:
                    return 4.59;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double SwerveXSDriveGearRatio(int gearRatioLevel, int pinionSize) {
            switch (gearRatioLevel) {
            case 1:
                switch (pinionSize) {
                case 12:
                    return 6;
                case 13:
                    return 5.54;
                case 14:
                    return 5.14;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 2:
                switch (pinionSize) {
                case 12:
                    return 4.71;
                case 13:
                    return 4.4;
                case 14:
                    return 4.13;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double SwerveX2DriveGearRatio(int gearRatioLevel, int pinionSize) {
            switch (gearRatioLevel) {
            case 1:
                switch (pinionSize) {
                case 10:
                    return 7.67;
                case 11:
                    return 6.98;
                case 12:
                    return 6.39;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 2:
                switch (pinionSize) {
                case 10:
                    return 6.82;
                case 11:
                    return 6.2;
                case 12:
                    return 5.68;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 3:
                switch (pinionSize) {
                case 10:
                    return 6.48;
                case 11:
                    return 5.89;
                case 12:
                    return 5.4;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 4:
                switch (pinionSize) {
                case 10:
                    return 5.67;
                case 11:
                    return 5.15;
                case 12:
                    return 4.73;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double SwerveX2SDriveGearRatio(int gearRatioLevel, int pinionSize) {
            switch (gearRatioLevel) {
            case 1:
                switch (pinionSize) {
                case 15:
                    return 6.0;
                case 16:
                    return 5.63;
                case 17:
                    return 5.29;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 2:
                switch (pinionSize) {
                case 17:
                    return 4.94;
                case 18:
                    return 4.67;
                case 19:
                    return 4.42;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            case 3:
                switch (pinionSize) {
                case 19:
                    return 4.11;
                case 20:
                    return 3.9;
                case 21:
                    return 3.71;
                default:
                    ThrowUnknownPinionSize(pinionSize);
                }
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double MAXSwerveDriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 5.5;
            case 2:
                return 5.08;
            case 3:
                return 4.71;
            case 4:
                return 4.50;
            case 5:
                return 4.29;
            case 6:
                return 4;
            case 7:
                return 3.75;
            case 8:
                return 3.56;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] double ThriftySwerveDriveGearRatio(int gearRatioLevel) {
            switch (gearRatioLevel) {
            case 1:
                return 6.75;
            case 2:
                return 6.23;
            case 3:
                return 5.79;
            case 4:
                return 6;
            case 5:
                return 5.54;
            case 6:
                return 5.14;
            default:
                ThrowUnknownGearingLevel(gearRatioLevel);
            }
        }

        [[nodiscard]] configs::GyroSimulationFactory MakeGyroFactory(double averageDriftingIn30SecsMotionlessDeg,
                                                                     double velocityMeasurementStandardDeviationPercent) {
            return [=] { return std::make_unique<GyroSimulation>(averageDriftingIn30SecsMotionlessDeg, velocityMeasurementStandardDeviationPercent); };
        }
    } // namespace

    configs::SwerveModuleSimulationConfig OfMark4(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, Mark4DriveGearRatio(gearRatioLevel), 12.8, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfMark4i(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, Mark4iDriveGearRatio(gearRatioLevel), 150.0 / 7.0, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfMark4n(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, Mark4nDriveGearRatio(gearRatioLevel), 18.75, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfMark5n(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, Mark5DriveGearRatio(gearRatioLevel), 287.0 / 11.0, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfMark5i(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, Mark5DriveGearRatio(gearRatioLevel), 26.0, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfSwerveX(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                    double firstStageRatio) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, firstStageRatio * SwerveXSecondStageRatio(gearRatioLevel), 11.3142, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfSwerveXFlipped(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                           int pinionSize) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, SwerveXFlippedDriveGearRatio(gearRatioLevel, pinionSize), 13.3714, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfSwerveXS(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                     int pinionSize) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, SwerveXSDriveGearRatio(gearRatioLevel, pinionSize), 41.25, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfSwerveX2(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                     int pinionSize) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, SwerveX2DriveGearRatio(gearRatioLevel, pinionSize), 12.1, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfSwerveX2S(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel,
                                                      int pinionSize) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, SwerveX2SDriveGearRatio(gearRatioLevel, pinionSize), 25.9, wheelCOF);
    }

    configs::SwerveModuleSimulationConfig OfMAXSwerve(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return configs::SwerveModuleSimulationConfig{driveMotor,
                                                     steerMotor,
                                                     MAXSwerveDriveGearRatio(gearRatioLevel),
                                                     9424.0 / 203.0,
                                                     wpi::units::volt_t{0.1},
                                                     wpi::units::volt_t{0.1},
                                                     wpi::units::inch_t{1.5},
                                                     wpi::units::kilogram_square_meter_t{0.02},
                                                     wheelCOF};
    }

    configs::SwerveModuleSimulationConfig OfThriftySwerve(wpi::math::DCMotor driveMotor, wpi::math::DCMotor steerMotor, double wheelCOF, int gearRatioLevel) {
        return MakeTypicalModuleConfig(driveMotor, steerMotor, ThriftySwerveDriveGearRatio(gearRatioLevel), 25, wheelCOF);
    }

    configs::GyroSimulationFactory OfPigeon2() {
        return MakeGyroFactory(0.5, 0.02);
    }

    configs::GyroSimulationFactory OfNav2X() {
        return MakeGyroFactory(2, 0.04);
    }

    configs::GyroSimulationFactory OfGenericGyro() {
        return MakeGyroFactory(5, 0.06);
    }
} // namespace maplesim::simulation::drivesims::COTS
