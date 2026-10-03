#pragma once

#include <concepts>
#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/torque.hpp>
#include <wpi/units/voltage.hpp>

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
                                                               const wpi::math::Rotation2d& robotFacing, wpi::units::newton_t gravityForceOnModule);
        /** As above, scaling the distance the drive wheel reports to odometry by driveWheelOdometryDistanceScale. */
        physics::Force2d UpdateSimulationSubTickGetModuleForce(const physics::LinearVelocity2d& moduleCurrentGroundVelocityWorldRelative,
                                                               const wpi::math::Rotation2d& robotFacing, wpi::units::newton_t gravityForceOnModule,
                                                               double driveWheelOdometryDistanceScale);

        [[nodiscard]] wpi::math::SwerveModuleVelocity GetCurrentState() const;

        void SetDriveWheelOdometryDistanceScale(double driveWheelOdometryDistanceScale);
        [[nodiscard]] double GetDriveWheelOdometryDistanceScale() const { return driveWheelOdometryDistanceScale_; }

        [[nodiscard]] wpi::units::volt_t GetDriveMotorAppliedVoltage() const { return driveMotorAppliedVoltage_; }
        [[nodiscard]] wpi::units::volt_t GetSteerMotorAppliedVoltage() const { return steerMotorSim_.GetAppliedVoltage(); }
        [[nodiscard]] wpi::units::ampere_t GetDriveMotorSupplyCurrent() const;
        [[nodiscard]] wpi::units::ampere_t GetDriveMotorStatorCurrent() const { return driveMotorStatorCurrent_; }
        [[nodiscard]] wpi::units::ampere_t GetSteerMotorSupplyCurrent() const { return steerMotorSim_.GetSupplyCurrent(); }
        [[nodiscard]] wpi::units::ampere_t GetSteerMotorStatorCurrent() const { return steerMotorSim_.GetStatorCurrent(); }

        [[nodiscard]] wpi::units::radian_t GetDriveEncoderUnGearedPosition() const { return GetDriveWheelFinalPosition() * config.driveGearRatio; }
        [[nodiscard]] wpi::units::radian_t GetDriveWheelFinalPosition() const { return driveWheelFinalPosition_; }
        [[nodiscard]] wpi::units::radians_per_second_t GetDriveEncoderUnGearedSpeed() const { return GetDriveWheelFinalSpeed() * config.driveGearRatio; }
        [[nodiscard]] wpi::units::radians_per_second_t GetDriveWheelFinalSpeed() const { return driveWheelFinalSpeed_; }

        [[nodiscard]] wpi::units::radian_t GetSteerRelativeEncoderPosition() const;
        [[nodiscard]] wpi::units::radians_per_second_t GetSteerRelativeEncoderVelocity() const {
            return GetSteerAbsoluteEncoderSpeed() * config.steerGearRatio;
        }
        [[nodiscard]] wpi::math::Rotation2d GetSteerAbsoluteFacing() const { return wpi::math::Rotation2d{GetSteerAbsoluteAngle()}; }
        [[nodiscard]] wpi::units::radian_t GetSteerAbsoluteAngle() const { return steerMotorSim_.GetAngularPosition(); }
        [[nodiscard]] wpi::units::radians_per_second_t GetSteerAbsoluteEncoderSpeed() const { return steerMotorSim_.GetVelocity(); }

        /** Each accessor below returns one reading per sub-tick of the last robot period, oldest first. */
        [[nodiscard]] std::vector<wpi::units::radian_t> GetCachedDriveEncoderUnGearedPositions() const;
        [[nodiscard]] std::vector<wpi::units::radian_t> GetCachedDriveWheelFinalPositions() const;
        [[nodiscard]] std::vector<wpi::units::radian_t> GetCachedSteerRelativeEncoderPositions() const;
        [[nodiscard]] std::vector<wpi::math::Rotation2d> GetCachedSteerAbsolutePositions() const;

        const configs::SwerveModuleSimulationConfig config;

    protected:
        friend class SwerveDriveSimulation;

        [[nodiscard]] wpi::math::SwerveModuleVelocity GetFreeSpinState() const;

    private:
        [[nodiscard]] physics::Force2d GetPropellingForce(wpi::units::newton_t grippingForce, const wpi::math::Rotation2d& moduleWorldFacing,
                                                          const physics::LinearVelocity2d& moduleCurrentGroundVelocity);
        [[nodiscard]] wpi::units::newton_meter_t GetDriveWheelTorque();
        void UpdateEncoderCaches(double driveWheelDistanceScale);

        motorsims::MapleMotorSim steerMotorSim_;
        wpi::units::volt_t driveMotorAppliedVoltage_{0.0};
        wpi::units::ampere_t driveMotorStatorCurrent_{0.0};
        wpi::units::radian_t driveWheelFinalPosition_{0.0};
        wpi::units::radians_per_second_t driveWheelFinalSpeed_{0.0};
        double driveWheelOdometryDistanceScale_ = 1.0;
        std::unique_ptr<motorsims::SimulatedMotorController> driveMotorController_;
        const wpi::units::radian_t steerRelativeEncoderOffSet_;
        std::deque<wpi::units::radian_t> driveWheelFinalPositionCache_;
        std::deque<wpi::units::radian_t> steerAbsoluteAngleCache_;
        motorsims::SimulatedBattery::ApplianceConnection driveMotorBatteryConnection_;
    };
} // namespace maplesim::simulation::drivesims
