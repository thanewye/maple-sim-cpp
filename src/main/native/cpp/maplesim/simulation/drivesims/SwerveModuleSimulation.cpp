#include "pch.h"

#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"

#include <cmath>
#include <limits>
#include <random>

#include <frc/MathUtil.h>
#include <units/math.h>

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation::drivesims {
    namespace {
        [[nodiscard]] double NextRandomDouble() {
            static std::mt19937_64 random{std::random_device{}()};
            return std::uniform_real_distribution<double>{0.0, 1.0}(random);
        }
    } // namespace

    SwerveModuleSimulation::SwerveModuleSimulation(configs::SwerveModuleSimulationConfig config)
        : config(std::move(config))
        , steerMotorSim_(this->config.steerMotorConfigs)
        , driveMotorController_(std::make_unique<motorsims::SimulatedMotorController::GenericMotorController>(this->config.driveMotorConfigs.motor))
        , steerRelativeEncoderOffSet_((NextRandomDouble() - 0.5) * 30)
        , driveWheelFinalPositionCache_(SimulatedArena::GetSimulationSubTicksIn1Period(), driveWheelFinalPosition_)
        , steerAbsolutePositionCache_(SimulatedArena::GetSimulationSubTicksIn1Period(), GetSteerAbsoluteFacing())
        , driveMotorBatteryConnection_(motorsims::SimulatedBattery::AddElectricalAppliances([this] { return GetDriveMotorSupplyCurrent(); })) {
        steerMotorSim_.UseSimpleDCMotorController();
    }

    motorsims::SimulatedMotorController::GenericMotorController& SwerveModuleSimulation::UseGenericMotorControllerForDrive() {
        return UseDriveMotorController(std::make_unique<motorsims::SimulatedMotorController::GenericMotorController>(config.driveMotorConfigs.motor));
    }

    motorsims::SimulatedMotorController::GenericMotorController& SwerveModuleSimulation::UseGenericControllerForSteer() {
        return steerMotorSim_.UseSimpleDCMotorController();
    }

    physics::Force2d SwerveModuleSimulation::UpdateSimulationSubTickGetModuleForce(const physics::LinearVelocity2d& moduleCurrentGroundVelocityWorldRelative,
                                                                                   const frc::Rotation2d& robotFacing, units::newton_t gravityForceOnModule) {
        steerMotorSim_.Update(SimulatedArena::GetSimulationDt());
        const units::newton_t grippingForce = config.GetGrippingForce(gravityForceOnModule);
        const frc::Rotation2d moduleWorldFacing = GetSteerAbsoluteFacing() + robotFacing;
        const physics::Force2d propellingForce = GetPropellingForce(grippingForce, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);
        UpdateEncoderCaches();
        return propellingForce;
    }

    physics::Force2d SwerveModuleSimulation::GetPropellingForce(units::newton_t grippingForce, const frc::Rotation2d& moduleWorldFacing,
                                                                const physics::LinearVelocity2d& moduleCurrentGroundVelocity) {
        const double wheelRadiusMeters = config.wheelRadius.value();
        const double grippingForceNewtons = grippingForce.value();
        const double driveWheelTorque = GetDriveWheelTorque().value();
        double propellingForceNewtons = driveWheelTorque / wheelRadiusMeters;
        const bool skidding = std::abs(propellingForceNewtons) > grippingForceNewtons;
        if (skidding) propellingForceNewtons = std::copysign(grippingForceNewtons, propellingForceNewtons);

        const units::radian_t angleBetweenGroundVelocityAndWheel = moduleWorldFacing.Radians() - moduleCurrentGroundVelocity.Angle().Radians();
        const double floorVelocityProjectionOnWheelDirectionMPS =
            moduleCurrentGroundVelocity.Norm().value() * units::math::cos(angleBetweenGroundVelocityAndWheel);
        driveWheelFinalSpeed_ = units::radians_per_second_t{floorVelocityProjectionOnWheelDirectionMPS / wheelRadiusMeters};

        if (skidding) {
            const units::radians_per_second_t skiddingEquilibriumWheelSpeed = config.driveMotorConfigs.CalculateMechanismVelocity(
                config.driveMotorConfigs.CalculateCurrent(units::newton_meter_t{propellingForceNewtons * wheelRadiusMeters}), driveMotorAppliedVoltage_);
            driveWheelFinalSpeed_ = driveWheelFinalSpeed_ * 0.5 + skiddingEquilibriumWheelSpeed * 0.5;
        }

        return physics::Force2d::FromPolar(units::newton_t{propellingForceNewtons}, moduleWorldFacing);
    }

    units::newton_meter_t SwerveModuleSimulation::GetDriveWheelTorque() {
        driveMotorAppliedVoltage_ = driveMotorController_->UpdateControlSignal(driveWheelFinalPosition_, driveWheelFinalSpeed_,
                                                                               GetDriveEncoderUnGearedPosition(), GetDriveEncoderUnGearedSpeed());
        driveMotorAppliedVoltage_ = motorsims::SimulatedBattery::Clamp(driveMotorAppliedVoltage_);
        driveMotorStatorCurrent_ = config.driveMotorConfigs.CalculateCurrent(driveWheelFinalSpeed_, driveMotorAppliedVoltage_);
        const units::newton_meter_t driveWheelTorque = config.driveMotorConfigs.CalculateTorque(driveMotorStatorCurrent_);
        return units::newton_meter_t{
            frc::ApplyDeadband(driveWheelTorque.value(), config.driveMotorConfigs.friction.value(), std::numeric_limits<double>::infinity())};
    }

    frc::SwerveModuleState SwerveModuleSimulation::GetCurrentState() const {
        return frc::SwerveModuleState{units::meters_per_second_t{GetDriveWheelFinalSpeed().value() * config.wheelRadius.value()}, GetSteerAbsoluteFacing()};
    }

    frc::SwerveModuleState SwerveModuleSimulation::GetFreeSpinState() const {
        const units::radians_per_second_t freeSpinWheelSpeed = config.driveMotorConfigs.CalculateMechanismVelocity(
            config.driveMotorConfigs.CalculateCurrent(config.driveMotorConfigs.friction), driveMotorAppliedVoltage_);
        return frc::SwerveModuleState{units::meters_per_second_t{freeSpinWheelSpeed.value() * config.wheelRadius.value()}, GetSteerAbsoluteFacing()};
    }

    void SwerveModuleSimulation::UpdateEncoderCaches() {
        driveWheelFinalPosition_ = driveWheelFinalPosition_ + driveWheelFinalSpeed_ * SimulatedArena::GetSimulationDt();
        steerAbsolutePositionCache_.pop_front();
        steerAbsolutePositionCache_.push_back(GetSteerAbsoluteFacing());
        driveWheelFinalPositionCache_.pop_front();
        driveWheelFinalPositionCache_.push_back(driveWheelFinalPosition_);
    }

    units::ampere_t SwerveModuleSimulation::GetDriveMotorSupplyCurrent() const {
        return GetDriveMotorStatorCurrent() * (driveMotorAppliedVoltage_ / motorsims::SimulatedBattery::GetBatteryVoltage());
    }

    units::radian_t SwerveModuleSimulation::GetSteerRelativeEncoderPosition() const {
        return GetSteerAbsoluteFacing().Radians() * config.steerGearRatio + steerRelativeEncoderOffSet_;
    }

    std::vector<units::radian_t> SwerveModuleSimulation::GetCachedDriveEncoderUnGearedPositions() const {
        std::vector<units::radian_t> positions;
        positions.reserve(driveWheelFinalPositionCache_.size());
        for (const units::radian_t driveWheelFinalPosition : driveWheelFinalPositionCache_)
            positions.push_back(driveWheelFinalPosition * config.driveGearRatio);
        return positions;
    }

    std::vector<units::radian_t> SwerveModuleSimulation::GetCachedDriveWheelFinalPositions() const {
        return {driveWheelFinalPositionCache_.begin(), driveWheelFinalPositionCache_.end()};
    }

    std::vector<units::radian_t> SwerveModuleSimulation::GetCachedSteerRelativeEncoderPositions() const {
        std::vector<units::radian_t> positions;
        positions.reserve(steerAbsolutePositionCache_.size());
        for (const frc::Rotation2d& absoluteFacing : steerAbsolutePositionCache_)
            positions.push_back(absoluteFacing.Radians() * config.steerGearRatio + steerRelativeEncoderOffSet_);
        return positions;
    }

    std::vector<frc::Rotation2d> SwerveModuleSimulation::GetCachedSteerAbsolutePositions() const {
        return {steerAbsolutePositionCache_.begin(), steerAbsolutePositionCache_.end()};
    }
} // namespace maplesim::simulation::drivesims
