#pragma once

#include <concepts>
#include <memory>
#include <utility>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "maplesim/simulation/motorsims/SimMotorConfigs.h"
#include "maplesim/simulation/motorsims/SimMotorState.h"
#include "maplesim/simulation/motorsims/SimulatedBattery.h"
#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

namespace maplesim::simulation::motorsims {
    /** DCMotorSim with a motor controller, current limiting, rotor friction and hard limits; draws current from SimulatedBattery while alive. */
    class MapleMotorSim {
    public:
        explicit MapleMotorSim(SimMotorConfigs configs);

        MapleMotorSim(const MapleMotorSim&) = delete;
        MapleMotorSim& operator=(const MapleMotorSim&) = delete;

        void Update(units::second_t dt);

        template<std::derived_from<SimulatedMotorController> T> T& UseMotorController(std::unique_ptr<T> motorController) {
            T& motorControllerRef = *motorController;
            controller_ = std::move(motorController);
            return motorControllerRef;
        }

        SimulatedMotorController::GenericMotorController& UseSimpleDCMotorController();

        [[nodiscard]] units::radian_t GetAngularPosition() const { return state_.mechanismAngularPosition; }
        [[nodiscard]] units::radian_t GetEncoderPosition() const { return GetAngularPosition() * configs_.gearing; }
        [[nodiscard]] units::radians_per_second_t GetVelocity() const { return state_.mechanismAngularVelocity; }
        [[nodiscard]] units::radians_per_second_t GetEncoderVelocity() const { return GetVelocity() * configs_.gearing; }
        [[nodiscard]] units::volt_t GetAppliedVoltage() const { return appliedVoltage_; }
        [[nodiscard]] units::ampere_t GetStatorCurrent() const { return statorCurrent_; }
        [[nodiscard]] units::ampere_t GetSupplyCurrent() const;
        [[nodiscard]] SimMotorConfigs& GetConfigs() { return configs_; }
        [[nodiscard]] const SimMotorConfigs& GetConfigs() const { return configs_; }

    private:
        SimMotorConfigs configs_;
        SimMotorState state_;
        std::unique_ptr<SimulatedMotorController> controller_;
        units::volt_t appliedVoltage_{0.0};
        units::ampere_t statorCurrent_{0.0};
        SimulatedBattery::ApplianceConnection batteryConnection_;
    };
} // namespace maplesim::simulation::motorsims
