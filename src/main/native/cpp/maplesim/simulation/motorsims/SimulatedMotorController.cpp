#include "pch.h"

#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

#include <cmath>
#include <limits>

namespace maplesim::simulation::motorsims {
    SimulatedMotorController::GenericMotorController::GenericMotorController(frc::DCMotor model)
        : model_(model)
        , forwardSoftwareLimit_(std::numeric_limits<double>::infinity())
        , reverseSoftwareLimit_(-std::numeric_limits<double>::infinity()) {}

    SimulatedMotorController::GenericMotorController& SimulatedMotorController::GenericMotorController::WithCurrentLimit(units::ampere_t currentLimit) {
        currentLimit_ = currentLimit;
        return *this;
    }

    SimulatedMotorController::GenericMotorController&
    SimulatedMotorController::GenericMotorController::WithSoftwareLimits(units::radian_t forwardSoftwareLimit, units::radian_t reverseSoftwareLimit) {
        forwardSoftwareLimit_ = forwardSoftwareLimit;
        reverseSoftwareLimit_ = reverseSoftwareLimit;
        return *this;
    }

    void SimulatedMotorController::GenericMotorController::RequestVoltage(units::volt_t voltage) {
        requestedVoltage_ = voltage;
    }

    units::volt_t SimulatedMotorController::GenericMotorController::ConstrainOutputVoltage(units::radian_t encoderAngle,
                                                                                           units::radians_per_second_t encoderVelocity,
                                                                                           units::volt_t requestedVoltage) const {
        const double kCurrentThreshold = 1.2;

        const double motorCurrentVelocityRadPerSec = encoderVelocity.value();
        const double currentLimitAmps = currentLimit_.value();
        const double requestedOutputVoltageVolts = requestedVoltage.value();
        const double currentAtRequestedVoltageAmps = model_.Current(encoderVelocity, requestedVoltage).value();

        double limitedVoltage = requestedOutputVoltageVolts;
        const bool currentTooHigh = std::abs(currentAtRequestedVoltageAmps) > (kCurrentThreshold * currentLimitAmps);
        if (currentTooHigh) {
            const units::ampere_t limitedCurrent{std::copysign(currentLimitAmps, currentAtRequestedVoltageAmps)};
            limitedVoltage = model_.Voltage(model_.Torque(limitedCurrent), units::radians_per_second_t{motorCurrentVelocityRadPerSec}).value();
        }

        if (std::abs(limitedVoltage) > std::abs(requestedOutputVoltageVolts)) limitedVoltage = requestedOutputVoltageVolts;

        if (encoderAngle >= forwardSoftwareLimit_ && limitedVoltage > 0) limitedVoltage = 0;
        if (encoderAngle <= reverseSoftwareLimit_ && limitedVoltage < 0) limitedVoltage = 0;

        return units::volt_t{limitedVoltage};
    }

    units::volt_t SimulatedMotorController::GenericMotorController::UpdateControlSignal(units::radian_t, units::radians_per_second_t,
                                                                                        units::radian_t encoderAngle,
                                                                                        units::radians_per_second_t encoderVelocity) {
        appliedVoltage_ = ConstrainOutputVoltage(encoderAngle, encoderVelocity, requestedVoltage_);
        return appliedVoltage_;
    }
} // namespace maplesim::simulation::motorsims
