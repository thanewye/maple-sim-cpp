#pragma once

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

namespace maplesim::simulation::motorsims {
    /** Computes the voltage a motor controller applies each sub-tick from the mechanism and encoder state. */
    class SimulatedMotorController {
    public:
        class GenericMotorController;

        virtual ~SimulatedMotorController() = default;

        virtual wpi::units::volt_t UpdateControlSignal(wpi::units::radian_t mechanismAngle, wpi::units::radians_per_second_t mechanismVelocity,
                                                       wpi::units::radian_t encoderAngle, wpi::units::radians_per_second_t encoderVelocity) = 0;
    };

    /** Applies a requested voltage, limited by stator current and software limits. */
    class SimulatedMotorController::GenericMotorController final : public SimulatedMotorController {
    public:
        explicit GenericMotorController(wpi::math::DCMotor model);

        GenericMotorController& WithCurrentLimit(wpi::units::ampere_t currentLimit);
        GenericMotorController& WithSoftwareLimits(wpi::units::radian_t forwardSoftwareLimit, wpi::units::radian_t reverseSoftwareLimit);

        void RequestVoltage(wpi::units::volt_t voltage);

        [[nodiscard]] wpi::units::volt_t ConstrainOutputVoltage(wpi::units::radian_t encoderAngle, wpi::units::radians_per_second_t encoderVelocity,
                                                                wpi::units::volt_t requestedVoltage) const;

        wpi::units::volt_t UpdateControlSignal(wpi::units::radian_t mechanismAngle, wpi::units::radians_per_second_t mechanismVelocity,
                                               wpi::units::radian_t encoderAngle, wpi::units::radians_per_second_t encoderVelocity) override;

        [[nodiscard]] wpi::units::volt_t GetAppliedVoltage() const { return appliedVoltage_; }

    private:
        wpi::math::DCMotor model_;
        wpi::units::ampere_t currentLimit_{150.0};
        wpi::units::radian_t forwardSoftwareLimit_;
        wpi::units::radian_t reverseSoftwareLimit_;

        wpi::units::volt_t requestedVoltage_{0.0};
        wpi::units::volt_t appliedVoltage_{0.0};
    };
} // namespace maplesim::simulation::motorsims
