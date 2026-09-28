#pragma once

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

namespace maplesim::simulation::motorsims {
    /** Computes the voltage a motor controller applies each sub-tick from the mechanism and encoder state. */
    class SimulatedMotorController {
    public:
        class GenericMotorController;

        virtual ~SimulatedMotorController() = default;

        virtual units::volt_t UpdateControlSignal(units::radian_t mechanismAngle, units::radians_per_second_t mechanismVelocity, units::radian_t encoderAngle,
                                                  units::radians_per_second_t encoderVelocity) = 0;
    };

    /** Applies a requested voltage, limited by stator current and software limits. */
    class SimulatedMotorController::GenericMotorController final : public SimulatedMotorController {
    public:
        explicit GenericMotorController(frc::DCMotor model);

        GenericMotorController& WithCurrentLimit(units::ampere_t currentLimit);
        GenericMotorController& WithSoftwareLimits(units::radian_t forwardSoftwareLimit, units::radian_t reverseSoftwareLimit);

        void RequestVoltage(units::volt_t voltage);

        [[nodiscard]] units::volt_t ConstrainOutputVoltage(units::radian_t encoderAngle, units::radians_per_second_t encoderVelocity,
                                                           units::volt_t requestedVoltage) const;

        units::volt_t UpdateControlSignal(units::radian_t mechanismAngle, units::radians_per_second_t mechanismVelocity, units::radian_t encoderAngle,
                                          units::radians_per_second_t encoderVelocity) override;

        [[nodiscard]] units::volt_t GetAppliedVoltage() const { return appliedVoltage_; }

    private:
        frc::DCMotor model_;
        units::ampere_t currentLimit_{150.0};
        units::radian_t forwardSoftwareLimit_;
        units::radian_t reverseSoftwareLimit_;

        units::volt_t requestedVoltage_{0.0};
        units::volt_t appliedVoltage_{0.0};
    };
} // namespace maplesim::simulation::motorsims
