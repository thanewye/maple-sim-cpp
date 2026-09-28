#pragma once

#include <concepts>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/force.h>
#include <units/torque.h>
#include <units/voltage.h>

#include "maplesim/physics/Vector2d.h"
#include "maplesim/simulation/drivesims/configs/SwerveModuleSimulationConfig.h"
#include "maplesim/simulation/motorsims/MapleMotorSim.h"
#include "maplesim/simulation/motorsims/SimMotorConfigs.h"
#include "maplesim/simulation/motorsims/SimulatedBattery.h"
#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

namespace maplesim::simulation::drivesims {
    class SwerveDriveSimulation;

    /** One swerve module: a steer motor sim plus a drive motor whose wheel is coupled to the chassis through tire friction. */
    class SwerveModuleSimulation {
    public:
        explicit SwerveModuleSimulation(configs::SwerveModuleSimulationConfig config);

        SwerveModuleSimulation(const SwerveModuleSimulation&) = delete;
        SwerveModuleSimulation& operator=(const SwerveModuleSimulation&) = delete;

        [[nodiscard]] const motorsims::SimMotorConfigs& GetDriveMotorConfigs() const { return config.driveMotorConfigs; }
        [[nodiscard]] motorsims::SimMotorConfigs& GetSteerMotorConfigs() { return steerMotorSim_.GetConfigs(); }

        template<std::derived_from<motorsims::SimulatedMotorController> T> T& UseDriveMotorController(std::unique_ptr<T> driveMotorController) {
            T& driveMotorControllerRef = *driveMotorController;
            driveMotorController_ = std::move(driveMotorController);
            return driveMotorControllerRef;
        }

        motorsims::SimulatedMotorController::GenericMotorController& UseGenericMotorControllerForDrive();

        template<std::derived_from<motorsims::SimulatedMotorController> T> T& UseSteerMotorController(std::unique_ptr<T> steerMotorController) {
            return steerMotorSim_.UseMotorController(std::move(steerMotorController));
        }

        motorsims::SimulatedMotorController::GenericMotorController& UseGenericControllerForSteer();

        /** Advances the steer motor, returns the world-relative force the wheel exerts on the chassis, and records encoder readings. */
        physics::Force2d UpdateSimulationSubTickGetModuleForce(const physics::LinearVelocity2d& moduleCurrentGroundVelocityWorldRelative,
                                                               const frc::Rotation2d& robotFacing, units::newton_t gravityForceOnModule);

        [[nodiscard]] frc::SwerveModuleState GetCurrentState() const;

        [[nodiscard]] units::volt_t GetDriveMotorAppliedVoltage() const { return driveMotorAppliedVoltage_; }
        [[nodiscard]] units::volt_t GetSteerMotorAppliedVoltage() const { return steerMotorSim_.GetAppliedVoltage(); }
        [[nodiscard]] units::ampere_t GetDriveMotorSupplyCurrent() const;
        [[nodiscard]] units::ampere_t GetDriveMotorStatorCurrent() const { return driveMotorStatorCurrent_; }
        [[nodiscard]] units::ampere_t GetSteerMotorSupplyCurrent() const { return steerMotorSim_.GetSupplyCurrent(); }
        [[nodiscard]] units::ampere_t GetSteerMotorStatorCurrent() const { return steerMotorSim_.GetStatorCurrent(); }

        [[nodiscard]] units::radian_t GetDriveEncoderUnGearedPosition() const { return GetDriveWheelFinalPosition() * config.driveGearRatio; }
        [[nodiscard]] units::radian_t GetDriveWheelFinalPosition() const { return driveWheelFinalPosition_; }
        [[nodiscard]] units::radians_per_second_t GetDriveEncoderUnGearedSpeed() const { return GetDriveWheelFinalSpeed() * config.driveGearRatio; }
        [[nodiscard]] units::radians_per_second_t GetDriveWheelFinalSpeed() const { return driveWheelFinalSpeed_; }

        [[nodiscard]] units::radian_t GetSteerRelativeEncoderPosition() const;
        [[nodiscard]] units::radians_per_second_t GetSteerRelativeEncoderVelocity() const { return GetSteerAbsoluteEncoderSpeed() * config.steerGearRatio; }
        [[nodiscard]] frc::Rotation2d GetSteerAbsoluteFacing() const { return frc::Rotation2d{GetSteerAbsoluteAngle()}; }
        [[nodiscard]] units::radian_t GetSteerAbsoluteAngle() const { return steerMotorSim_.GetAngularPosition(); }
        [[nodiscard]] units::radians_per_second_t GetSteerAbsoluteEncoderSpeed() const { return steerMotorSim_.GetVelocity(); }

        /** Each accessor below returns one reading per sub-tick of the last robot period, oldest first. */
        [[nodiscard]] std::vector<units::radian_t> GetCachedDriveEncoderUnGearedPositions() const;
        [[nodiscard]] std::vector<units::radian_t> GetCachedDriveWheelFinalPositions() const;
        [[nodiscard]] std::vector<units::radian_t> GetCachedSteerRelativeEncoderPositions() const;
        [[nodiscard]] std::vector<frc::Rotation2d> GetCachedSteerAbsolutePositions() const;

        const configs::SwerveModuleSimulationConfig config;

    protected:
        friend class SwerveDriveSimulation;

        [[nodiscard]] frc::SwerveModuleState GetFreeSpinState() const;

    private:
        [[nodiscard]] physics::Force2d GetPropellingForce(units::newton_t grippingForce, const frc::Rotation2d& moduleWorldFacing,
                                                          const physics::LinearVelocity2d& moduleCurrentGroundVelocity);
        [[nodiscard]] units::newton_meter_t GetDriveWheelTorque();
        void UpdateEncoderCaches();

        motorsims::MapleMotorSim steerMotorSim_;
        units::volt_t driveMotorAppliedVoltage_{0.0};
        units::ampere_t driveMotorStatorCurrent_{0.0};
        units::radian_t driveWheelFinalPosition_{0.0};
        units::radians_per_second_t driveWheelFinalSpeed_{0.0};
        std::unique_ptr<motorsims::SimulatedMotorController> driveMotorController_;
        const units::radian_t steerRelativeEncoderOffSet_;
        std::deque<units::radian_t> driveWheelFinalPositionCache_;
        std::deque<frc::Rotation2d> steerAbsolutePositionCache_;
        motorsims::SimulatedBattery::ApplianceConnection driveMotorBatteryConnection_;
    };
} // namespace maplesim::simulation::drivesims
