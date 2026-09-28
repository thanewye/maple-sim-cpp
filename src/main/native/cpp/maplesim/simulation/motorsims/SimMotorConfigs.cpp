#include "pch.h"

#include "maplesim/simulation/motorsims/SimMotorConfigs.h"

#include <limits>

namespace maplesim::simulation::motorsims {
    SimMotorConfigs::SimMotorConfigs(frc::DCMotor motor, double gearing, units::kilogram_square_meter_t loadMOI, units::volt_t frictionVoltage)
        : motor(motor)
        , gearing(gearing)
        , loadMOI(loadMOI)
        , friction(motor.Torque(motor.Current(units::radians_per_second_t{0.0}, frictionVoltage)))
        , forwardHardwareLimit_(std::numeric_limits<double>::infinity())
        , reverseHardwareLimit_(-std::numeric_limits<double>::infinity()) {}

    units::volt_t SimMotorConfigs::CalculateVoltage(units::ampere_t current, units::radians_per_second_t mechanismVelocity) const {
        return motor.Voltage(motor.Torque(current), mechanismVelocity * gearing);
    }

    units::radians_per_second_t SimMotorConfigs::CalculateMechanismVelocity(units::ampere_t current, units::volt_t voltage) const {
        return motor.Speed(motor.Torque(current), voltage) / gearing;
    }

    units::ampere_t SimMotorConfigs::CalculateCurrent(units::radians_per_second_t mechanismVelocity, units::volt_t voltage) const {
        return motor.Current(mechanismVelocity * gearing, voltage);
    }

    units::ampere_t SimMotorConfigs::CalculateCurrent(units::newton_meter_t torque) const {
        return motor.Current(torque / gearing);
    }

    units::newton_meter_t SimMotorConfigs::CalculateTorque(units::ampere_t current) const {
        return motor.Torque(current) * gearing;
    }

    SimMotorConfigs& SimMotorConfigs::WithHardLimits(units::radian_t forwardLimit, units::radian_t reverseLimit) {
        forwardHardwareLimit_ = forwardLimit;
        reverseHardwareLimit_ = reverseLimit;
        return *this;
    }

    units::radians_per_second_t SimMotorConfigs::FreeSpinMechanismVelocity() const {
        return motor.freeSpeed / gearing;
    }

    units::ampere_t SimMotorConfigs::FreeSpinCurrent() const {
        return motor.freeCurrent;
    }

    units::ampere_t SimMotorConfigs::StallCurrent() const {
        return motor.stallCurrent;
    }

    units::newton_meter_t SimMotorConfigs::StallTorque() const {
        return motor.stallTorque;
    }

    units::volt_t SimMotorConfigs::NominalVoltage() const {
        return motor.nominalVoltage;
    }
} // namespace maplesim::simulation::motorsims
