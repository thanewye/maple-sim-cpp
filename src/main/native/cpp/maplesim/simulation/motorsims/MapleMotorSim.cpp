#include "pch.h"

#include "maplesim/simulation/motorsims/MapleMotorSim.h"

namespace maplesim::simulation::motorsims {
    namespace {
        class ZeroVoltageController final : public SimulatedMotorController {
        public:
            units::volt_t UpdateControlSignal(units::radian_t, units::radians_per_second_t, units::radian_t, units::radians_per_second_t) override {
                return units::volt_t{0.0};
            }
        };
    } // namespace

    MapleMotorSim::MapleMotorSim(SimMotorConfigs configs)
        : configs_(configs)
        , controller_(std::make_unique<ZeroVoltageController>())
        , batteryConnection_(SimulatedBattery::AddMotor(*this)) {}

    void MapleMotorSim::Update(units::second_t dt) {
        appliedVoltage_ =
            controller_->UpdateControlSignal(state_.mechanismAngularPosition, state_.mechanismAngularVelocity,
                                             state_.mechanismAngularPosition * configs_.gearing, state_.mechanismAngularVelocity * configs_.gearing);
        appliedVoltage_ = SimulatedBattery::Clamp(appliedVoltage_);
        statorCurrent_ = configs_.CalculateCurrent(state_.mechanismAngularVelocity, appliedVoltage_);
        state_.Step(configs_.CalculateTorque(statorCurrent_), configs_.friction, configs_.loadMOI, dt);

        if (state_.mechanismAngularPosition <= configs_.reverseHardwareLimit_)
            state_ = SimMotorState{configs_.reverseHardwareLimit_, units::radians_per_second_t{0.0}};
        else if (state_.mechanismAngularPosition >= configs_.forwardHardwareLimit_)
            state_ = SimMotorState{configs_.forwardHardwareLimit_, units::radians_per_second_t{0.0}};
    }

    SimulatedMotorController::GenericMotorController& MapleMotorSim::UseSimpleDCMotorController() {
        return UseMotorController(std::make_unique<SimulatedMotorController::GenericMotorController>(configs_.motor));
    }

    units::ampere_t MapleMotorSim::GetSupplyCurrent() const {
        return GetStatorCurrent() * (appliedVoltage_ / SimulatedBattery::GetBatteryVoltage());
    }
} // namespace maplesim::simulation::motorsims
