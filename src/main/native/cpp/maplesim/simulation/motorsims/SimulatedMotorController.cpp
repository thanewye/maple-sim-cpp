#include "pch.h"

#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

#include <cmath>
#include <limits>

namespace maplesim::simulation::motorsims {
    SimulatedMotorController::GenericMotorController::GenericMotorController(wpi::math::DCMotor model)
        : model_(model)
        , forwardSoftwareLimit_(std::numeric_limits<double>::infinity())
        , reverseSoftwareLimit_(-std::numeric_limits<double>::infinity()) {}

    SimulatedMotorController::GenericMotorController& SimulatedMotorController::GenericMotorController::WithCurrentLimit(wpi::units::ampere_t currentLimit) {
        currentLimit_ = currentLimit;
        return *this;
    }

    SimulatedMotorController::GenericMotorController&
    SimulatedMotorController::GenericMotorController::WithSoftwareLimits(wpi::units::radian_t forwardSoftwareLimit, wpi::units::radian_t reverseSoftwareLimit) {
        forwardSoftwareLimit_ = forwardSoftwareLimit;
        reverseSoftwareLimit_ = reverseSoftwareLimit;
        return *this;
    }

    void SimulatedMotorController::GenericMotorController::RequestVoltage(wpi::units::volt_t voltage) {
        requestedVoltage_ = voltage;
    }

    wpi::units::volt_t SimulatedMotorController::GenericMotorController::ConstrainOutputVoltage(wpi::units::radian_t encoderAngle,
                                                                                                wpi::units::radians_per_second_t encoderVelocity,
                                                                                                wpi::units::volt_t requestedVoltage) const {
        const double kCurrentThreshold = 1.2;

        const double motorCurrentVelocityRadPerSec = encoderVelocity.value();
        const double currentLimitAmps = currentLimit_.value();
        const double requestedOutputVoltageVolts = requestedVoltage.value();
        const double currentAtRequestedVoltageAmps = model_.Current(encoderVelocity, requestedVoltage).value();

        double limitedVoltage = requestedOutputVoltageVolts;
        const bool currentTooHigh = std::abs(currentAtRequestedVoltageAmps) > (kCurrentThreshold * currentLimitAmps);
        if (currentTooHigh) {
            const wpi::units::ampere_t limitedCurrent{std::copysign(currentLimitAmps, currentAtRequestedVoltageAmps)};
            limitedVoltage = model_.Voltage(model_.Torque(limitedCurrent), wpi::units::radians_per_second_t{motorCurrentVelocityRadPerSec}).value();
        }

        if (std::abs(limitedVoltage) > std::abs(requestedOutputVoltageVolts)) limitedVoltage = requestedOutputVoltageVolts;

        if (encoderAngle >= forwardSoftwareLimit_ && limitedVoltage > 0) limitedVoltage = 0;
        if (encoderAngle <= reverseSoftwareLimit_ && limitedVoltage < 0) limitedVoltage = 0;

        return wpi::units::volt_t{limitedVoltage};
    }

    wpi::units::volt_t SimulatedMotorController::GenericMotorController::UpdateControlSignal(wpi::units::radian_t, wpi::units::radians_per_second_t,
                                                                                             wpi::units::radian_t encoderAngle,
                                                                                             wpi::units::radians_per_second_t encoderVelocity) {
        appliedVoltage_ = ConstrainOutputVoltage(encoderAngle, encoderVelocity, requestedVoltage_);
        return appliedVoltage_;
    }
} // namespace maplesim::simulation::motorsims
