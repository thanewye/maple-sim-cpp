#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/voltage.h>

namespace maplesim::simulation::motorsims {
    class MapleMotorSim;

    /** Motor model, gearing, load inertia, friction and hard limits of a simulated mechanism. */
    class SimMotorConfigs {
    public:
        SimMotorConfigs(frc::DCMotor motor, double gearing, units::kilogram_square_meter_t loadMOI, units::volt_t frictionVoltage);

        [[nodiscard]] units::volt_t CalculateVoltage(units::ampere_t current, units::radians_per_second_t mechanismVelocity) const;
        [[nodiscard]] units::radians_per_second_t CalculateMechanismVelocity(units::ampere_t current, units::volt_t voltage) const;
        [[nodiscard]] units::ampere_t CalculateCurrent(units::radians_per_second_t mechanismVelocity, units::volt_t voltage) const;
        [[nodiscard]] units::ampere_t CalculateCurrent(units::newton_meter_t torque) const;
        [[nodiscard]] units::newton_meter_t CalculateTorque(units::ampere_t current) const;

        SimMotorConfigs& WithHardLimits(units::radian_t forwardLimit, units::radian_t reverseLimit);

        [[nodiscard]] units::radians_per_second_t FreeSpinMechanismVelocity() const;
        [[nodiscard]] units::ampere_t FreeSpinCurrent() const;
        [[nodiscard]] units::ampere_t StallCurrent() const;
        [[nodiscard]] units::newton_meter_t StallTorque() const;
        [[nodiscard]] units::volt_t NominalVoltage() const;

        frc::DCMotor motor;
        double gearing;
        units::kilogram_square_meter_t loadMOI;
        units::newton_meter_t friction;

    private:
        friend class MapleMotorSim;

        units::radian_t forwardHardwareLimit_;
        units::radian_t reverseHardwareLimit_;
    };
} // namespace maplesim::simulation::motorsims
