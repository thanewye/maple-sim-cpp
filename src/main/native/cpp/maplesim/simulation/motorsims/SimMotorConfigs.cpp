#include "pch.h"

#include "maplesim/simulation/motorsims/SimMotorConfigs.h"

#include <limits>

namespace maplesim::simulation::motorsims {
    SimMotorConfigs::SimMotorConfigs(wpi::math::DCMotor motor, double gearing, wpi::units::kilogram_square_meter_t loadMOI, wpi::units::volt_t frictionVoltage)
        : motor(motor)
        , gearing(gearing)
        , loadMOI(loadMOI)
        , friction(motor.Torque(motor.Current(wpi::units::radians_per_second_t{0.0}, frictionVoltage)))
        , forwardHardwareLimit_(std::numeric_limits<double>::infinity())
        , reverseHardwareLimit_(-std::numeric_limits<double>::infinity()) {}

    wpi::units::volt_t SimMotorConfigs::CalculateVoltage(wpi::units::ampere_t current, wpi::units::radians_per_second_t mechanismVelocity) const {
        return motor.Voltage(motor.Torque(current), mechanismVelocity * gearing);
    }

    wpi::units::radians_per_second_t SimMotorConfigs::CalculateMechanismVelocity(wpi::units::ampere_t current, wpi::units::volt_t voltage) const {
        return motor.Velocity(motor.Torque(current), voltage) / gearing;
    }

    wpi::units::ampere_t SimMotorConfigs::CalculateCurrent(wpi::units::radians_per_second_t mechanismVelocity, wpi::units::volt_t voltage) const {
        return motor.Current(mechanismVelocity * gearing, voltage);
    }

    wpi::units::ampere_t SimMotorConfigs::CalculateCurrent(wpi::units::newton_meter_t torque) const {
        return motor.Current(torque / gearing);
    }

    wpi::units::newton_meter_t SimMotorConfigs::CalculateTorque(wpi::units::ampere_t current) const {
        return motor.Torque(current) * gearing;
    }

    SimMotorConfigs& SimMotorConfigs::WithHardLimits(wpi::units::radian_t forwardLimit, wpi::units::radian_t reverseLimit) {
        forwardHardwareLimit_ = forwardLimit;
        reverseHardwareLimit_ = reverseLimit;
        return *this;
    }

    wpi::units::radians_per_second_t SimMotorConfigs::FreeSpinMechanismVelocity() const {
        return motor.freeSpeed / gearing;
    }

    wpi::units::ampere_t SimMotorConfigs::FreeSpinCurrent() const {
        return motor.freeCurrent;
    }

    wpi::units::ampere_t SimMotorConfigs::StallCurrent() const {
        return motor.stallCurrent;
    }

    wpi::units::newton_meter_t SimMotorConfigs::StallTorque() const {
        return motor.stallTorque;
    }

    wpi::units::volt_t SimMotorConfigs::NominalVoltage() const {
        return motor.nominalVoltage;
    }
} // namespace maplesim::simulation::motorsims
