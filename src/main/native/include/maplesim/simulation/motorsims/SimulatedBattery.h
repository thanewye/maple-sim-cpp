#pragma once

#include <cstdint>
#include <functional>

#include <wpi/units/current.hpp>
#include <wpi/units/voltage.hpp>

namespace maplesim::simulation::motorsims {
    class MapleMotorSim;

    /** Simulates the main battery sagging under the current drawn by every connected appliance. */
    class SimulatedBattery {
    public:
        /** Disconnects its appliance from the battery on destruction. */
        class ApplianceConnection {
        public:
            ApplianceConnection() = default;
            explicit ApplianceConnection(std::uint64_t id);
            ~ApplianceConnection();

            ApplianceConnection(ApplianceConnection&& other) noexcept;
            ApplianceConnection& operator=(ApplianceConnection&& other) noexcept;
            ApplianceConnection(const ApplianceConnection&) = delete;
            ApplianceConnection& operator=(const ApplianceConnection&) = delete;

            void Disconnect();

        private:
            std::uint64_t id_ = 0;
            bool connected_ = false;
        };

        SimulatedBattery() = delete;

        [[nodiscard]] static ApplianceConnection AddElectricalAppliances(std::function<wpi::units::ampere_t()> customElectricalAppliances);
        [[nodiscard]] static ApplianceConnection AddMotor(const MapleMotorSim& mapleMotorSim);

        static void SimulationSubTick();

        [[nodiscard]] static wpi::units::volt_t GetBatteryVoltage();
        [[nodiscard]] static wpi::units::ampere_t GetTotalCurrentDrawn();
        [[nodiscard]] static wpi::units::volt_t Clamp(wpi::units::volt_t voltage);
    };
} // namespace maplesim::simulation::motorsims
