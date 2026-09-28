#include "pch.h"

#include "maplesim/simulation/motorsims/SimulatedBattery.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include <frc/filter/LinearFilter.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <hal/DriverStation.h>
#include <units/impedance.h>

#include "maplesim/simulation/motorsims/MapleMotorSim.h"

namespace maplesim::simulation::motorsims {
    namespace {
        constexpr double kBatteryNominalVoltage = 13.5;

        struct BatteryState {
            frc::LinearFilter<double> currentFilter = frc::LinearFilter<double>::MovingAverage(50);
            std::map<std::uint64_t, std::function<units::ampere_t()>> electricalAppliances;
            std::uint64_t nextApplianceId = 0;
            double batteryVoltageVolts = kBatteryNominalVoltage;
        };

        /** Mirrors Java's DriverStation.reportError(message, false), which skips the per-call stack-trace symbolization. */
        void ReportErrorWithoutStackTrace(const char* message) {
            HAL_SendError(true, 1, false, message, "SimulatedBattery::SimulationSubTick", "", true);
        }

        [[nodiscard]] BatteryState& GetBatteryState() {
            static BatteryState state;
            return state;
        }
    } // namespace

    SimulatedBattery::ApplianceConnection::ApplianceConnection(std::uint64_t id)
        : id_(id)
        , connected_(true) {}

    SimulatedBattery::ApplianceConnection::~ApplianceConnection() {
        Disconnect();
    }

    SimulatedBattery::ApplianceConnection::ApplianceConnection(ApplianceConnection&& other) noexcept
        : id_(other.id_)
        , connected_(std::exchange(other.connected_, false)) {}

    SimulatedBattery::ApplianceConnection& SimulatedBattery::ApplianceConnection::operator=(ApplianceConnection&& other) noexcept {
        if (this != &other) {
            Disconnect();
            id_ = other.id_;
            connected_ = std::exchange(other.connected_, false);
        }
        return *this;
    }

    void SimulatedBattery::ApplianceConnection::Disconnect() {
        if (std::exchange(connected_, false)) GetBatteryState().electricalAppliances.erase(id_);
    }

    SimulatedBattery::ApplianceConnection SimulatedBattery::AddElectricalAppliances(std::function<units::ampere_t()> customElectricalAppliances) {
        BatteryState& state = GetBatteryState();
        const std::uint64_t id = state.nextApplianceId++;
        state.electricalAppliances.emplace(id, std::move(customElectricalAppliances));
        return ApplianceConnection{id};
    }

    SimulatedBattery::ApplianceConnection SimulatedBattery::AddMotor(const MapleMotorSim& mapleMotorSim) {
        return AddElectricalAppliances([&mapleMotorSim] { return mapleMotorSim.GetSupplyCurrent(); });
    }

    void SimulatedBattery::SimulationSubTick() {
        BatteryState& state = GetBatteryState();
        double totalCurrentAmps = GetTotalCurrentDrawn().value();
        totalCurrentAmps = state.currentFilter.Calculate(totalCurrentAmps);

        state.batteryVoltageVolts =
            frc::sim::BatterySim::Calculate(units::volt_t{kBatteryNominalVoltage}, units::ohm_t{0.02}, {units::ampere_t{totalCurrentAmps}}).value();

        if (std::isnan(state.batteryVoltageVolts) || std::isnan(totalCurrentAmps)) {
            state.batteryVoltageVolts = 12.0;
            ReportErrorWithoutStackTrace("[MapleSim] Internal Library Error: Calculated battery voltage is invalid, reverting to normal operation voltage...");
        }
        const double brownoutVoltageVolts = frc::sim::RoboRioSim::GetBrownoutVoltage().value();
        if (state.batteryVoltageVolts < brownoutVoltageVolts) {
            state.batteryVoltageVolts = brownoutVoltageVolts;
            ReportErrorWithoutStackTrace("[MapleSim] BrownOut Detected, protecting battery voltage...");
        }

        frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{state.batteryVoltageVolts});

        frc::SmartDashboard::PutNumber("BatterySim/TotalCurrent (Amps)", totalCurrentAmps);
        frc::SmartDashboard::PutNumber("BatterySim/BatteryVoltage (Volts)", state.batteryVoltageVolts);
    }

    units::volt_t SimulatedBattery::GetBatteryVoltage() {
        return units::volt_t{GetBatteryState().batteryVoltageVolts};
    }

    units::ampere_t SimulatedBattery::GetTotalCurrentDrawn() {
        double totalCurrentAmps = 0.0;
        for (const auto& [id, currentSupplier] : GetBatteryState().electricalAppliances)
            totalCurrentAmps += currentSupplier().value();
        return units::ampere_t{totalCurrentAmps};
    }

    units::volt_t SimulatedBattery::Clamp(units::volt_t voltage) {
        const double batteryVoltageVolts = GetBatteryState().batteryVoltageVolts;
        return units::volt_t{std::clamp(voltage.value(), -batteryVoltageVolts, batteryVoltageVolts)};
    }
} // namespace maplesim::simulation::motorsims
