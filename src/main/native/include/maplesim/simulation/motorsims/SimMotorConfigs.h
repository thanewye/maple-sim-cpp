#pragma once

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/torque.hpp>
#include <wpi/units/voltage.hpp>

namespace maplesim::simulation::motorsims {
    class MapleMotorSim;

    /** Motor model, gearing, load inertia, friction and hard limits of a simulated mechanism. */
    class SimMotorConfigs {
    public:
        SimMotorConfigs(wpi::math::DCMotor motor, double gearing, wpi::units::kilogram_square_meter_t loadMOI, wpi::units::volt_t frictionVoltage);

        [[nodiscard]] wpi::units::volt_t CalculateVoltage(wpi::units::ampere_t current, wpi::units::radians_per_second_t mechanismVelocity) const;
        [[nodiscard]] wpi::units::radians_per_second_t CalculateMechanismVelocity(wpi::units::ampere_t current, wpi::units::volt_t voltage) const;
        [[nodiscard]] wpi::units::ampere_t CalculateCurrent(wpi::units::radians_per_second_t mechanismVelocity, wpi::units::volt_t voltage) const;
        [[nodiscard]] wpi::units::ampere_t CalculateCurrent(wpi::units::newton_meter_t torque) const;
        [[nodiscard]] wpi::units::newton_meter_t CalculateTorque(wpi::units::ampere_t current) const;

        SimMotorConfigs& WithHardLimits(wpi::units::radian_t forwardLimit, wpi::units::radian_t reverseLimit);

        [[nodiscard]] wpi::units::radians_per_second_t FreeSpinMechanismVelocity() const;
        [[nodiscard]] wpi::units::ampere_t FreeSpinCurrent() const;
        [[nodiscard]] wpi::units::ampere_t StallCurrent() const;
        [[nodiscard]] wpi::units::newton_meter_t StallTorque() const;
        [[nodiscard]] wpi::units::volt_t NominalVoltage() const;

        wpi::math::DCMotor motor;
        double gearing;
        wpi::units::kilogram_square_meter_t loadMOI;
        wpi::units::newton_meter_t friction;

    private:
        friend class MapleMotorSim;

        wpi::units::radian_t forwardHardwareLimit_;
        wpi::units::radian_t reverseHardwareLimit_;
    };
} // namespace maplesim::simulation::motorsims
