#include "pch.h"

#include "maplesim/simulation/motorsims/SimulatedBattery.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

#include <wpi/hal/DriverStation.h>
#include <wpi/math/filter/LinearFilter.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/simulation/BatterySim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/units/impedance.hpp>

#include "maplesim/simulation/motorsims/MapleMotorSim.h"

namespace maplesim::simulation::motorsims {
    namespace {
        constexpr double kBatteryNominalVoltage = 13.5;

        struct BatteryState {
            wpi::math::LinearFilter<double> currentFilter = wpi::math::LinearFilter<double>::MovingAverage(50);
            std::map<std::uint64_t, std::function<wpi::units::ampere_t()>> electricalAppliances;
            std::uint64_t nextApplianceId = 0;
            double batteryVoltageVolts = kBatteryNominalVoltage;
        };

        /** Mirrors Java's DriverStation.reportError(message, false), which skips the per-call stack-trace symbolization. */
        void ReportErrorWithoutStackTrace(const char* message) {
            WPI_String details;
            WPI_String location;
            WPI_String callStack;
            WPI_InitString(&details, message);
            WPI_InitString(&location, "SimulatedBattery::SimulationSubTick");
            WPI_InitString(&callStack, "");
            HAL_SendError(true, 1, &details, &location, &callStack, true);
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

    SimulatedBattery::ApplianceConnection SimulatedBattery::AddElectricalAppliances(std::function<wpi::units::ampere_t()> customElectricalAppliances) {
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
            wpi::sim::BatterySim::Calculate(wpi::units::volt_t{kBatteryNominalVoltage}, wpi::units::ohm_t{0.02}, {wpi::units::ampere_t{totalCurrentAmps}})
                .value();

        if (std::isnan(state.batteryVoltageVolts) || std::isnan(totalCurrentAmps)) {
            state.batteryVoltageVolts = 12.0;
            ReportErrorWithoutStackTrace("[MapleSim] Internal Library Error: Calculated battery voltage is invalid, reverting to normal operation voltage...");
        }
        const double brownoutVoltageVolts = wpi::sim::RoboRioSim::GetBrownoutVoltage().value();
        if (state.batteryVoltageVolts < brownoutVoltageVolts) {
            state.batteryVoltageVolts = brownoutVoltageVolts;
            ReportErrorWithoutStackTrace("[MapleSim] BrownOut Detected, protecting battery voltage...");
        }

        wpi::sim::RoboRioSim::SetVInVoltage(wpi::units::volt_t{state.batteryVoltageVolts});

        wpi::nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("BatterySim/TotalCurrent (Amps)", totalCurrentAmps);
        wpi::nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->PutNumber("BatterySim/BatteryVoltage (Volts)", state.batteryVoltageVolts);
    }

    wpi::units::volt_t SimulatedBattery::GetBatteryVoltage() {
        return wpi::units::volt_t{GetBatteryState().batteryVoltageVolts};
    }

    wpi::units::ampere_t SimulatedBattery::GetTotalCurrentDrawn() {
        double totalCurrentAmps = 0.0;
        for (const auto& [id, currentSupplier] : GetBatteryState().electricalAppliances)
            totalCurrentAmps += currentSupplier().value();
        return wpi::units::ampere_t{totalCurrentAmps};
    }

    wpi::units::volt_t SimulatedBattery::Clamp(wpi::units::volt_t voltage) {
        const double batteryVoltageVolts = GetBatteryState().batteryVoltageVolts;
        return wpi::units::volt_t{std::clamp(voltage.value(), -batteryVoltageVolts, batteryVoltageVolts)};
    }
} // namespace maplesim::simulation::motorsims
