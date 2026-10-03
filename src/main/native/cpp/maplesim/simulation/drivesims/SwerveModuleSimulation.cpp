#include "pch.h"

#include "maplesim/simulation/drivesims/SwerveModuleSimulation.h"

#include <cmath>
#include <limits>
#include <random>

#include <wpi/math/util/MathUtil.hpp>
#include <wpi/units/math.hpp>

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation::drivesims {
    namespace {
        [[nodiscard]] double NextRandomDouble() {
            static std::mt19937_64 random{std::random_device{}()};
            return std::uniform_real_distribution<double>{0.0, 1.0}(random);
        }

        [[nodiscard]] double SanitizeDriveWheelOdometryScale(double requestedScale) {
            if (!std::isfinite(requestedScale) || requestedScale <= 0.0) return 1.0;
            return requestedScale;
        }
    } // namespace

    SwerveModuleSimulation::SwerveModuleSimulation(configs::SwerveModuleSimulationConfig config)
        : config(std::move(config))
        , steerMotorSim_(this->config.steerMotorConfigs)
        , driveMotorController_(std::make_unique<motorsims::SimulatedMotorController::GenericMotorController>(this->config.driveMotorConfigs.motor))
        , steerRelativeEncoderOffSet_((NextRandomDouble() - 0.5) * 30)
        , driveWheelFinalPositionCache_(SimulatedArena::GetSimulationSubTicksIn1Period(), driveWheelFinalPosition_)
        , steerAbsoluteAngleCache_(SimulatedArena::GetSimulationSubTicksIn1Period(), GetSteerAbsoluteAngle())
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
                                                                                   const wpi::math::Rotation2d& robotFacing,
                                                                                   wpi::units::newton_t gravityForceOnModule) {
        return UpdateSimulationSubTickGetModuleForce(moduleCurrentGroundVelocityWorldRelative, robotFacing, gravityForceOnModule,
                                                     driveWheelOdometryDistanceScale_);
    }

    physics::Force2d SwerveModuleSimulation::UpdateSimulationSubTickGetModuleForce(const physics::LinearVelocity2d& moduleCurrentGroundVelocityWorldRelative,
                                                                                   const wpi::math::Rotation2d& robotFacing,
                                                                                   wpi::units::newton_t gravityForceOnModule,
                                                                                   double driveWheelOdometryDistanceScale) {
        driveWheelOdometryDistanceScale_ = SanitizeDriveWheelOdometryScale(driveWheelOdometryDistanceScale);
        steerMotorSim_.Update(SimulatedArena::GetSimulationDt());
        const wpi::units::newton_t grippingForce = config.GetGrippingForce(gravityForceOnModule);
        const wpi::math::Rotation2d moduleWorldFacing = GetSteerAbsoluteFacing() + robotFacing;
        const physics::Force2d propellingForce = GetPropellingForce(grippingForce, moduleWorldFacing, moduleCurrentGroundVelocityWorldRelative);
        UpdateEncoderCaches(driveWheelOdometryDistanceScale_);
        return propellingForce;
    }

    physics::Force2d SwerveModuleSimulation::GetPropellingForce(wpi::units::newton_t grippingForce, const wpi::math::Rotation2d& moduleWorldFacing,
                                                                const physics::LinearVelocity2d& moduleCurrentGroundVelocity) {
        const double wheelRadiusMeters = config.wheelRadius.value();
        const double grippingForceNewtons = grippingForce.value();
        const double driveWheelTorque = GetDriveWheelTorque().value();
        double propellingForceNewtons = driveWheelTorque / wheelRadiusMeters;
        const bool skidding = std::abs(propellingForceNewtons) > grippingForceNewtons;
        if (skidding) propellingForceNewtons = std::copysign(grippingForceNewtons, propellingForceNewtons);

        const wpi::units::radian_t angleBetweenGroundVelocityAndWheel = moduleWorldFacing.Radians() - moduleCurrentGroundVelocity.Angle().Radians();
        const double floorVelocityProjectionOnWheelDirectionMPS =
            moduleCurrentGroundVelocity.Norm().value() * wpi::units::math::cos(angleBetweenGroundVelocityAndWheel);
        driveWheelFinalSpeed_ = wpi::units::radians_per_second_t{floorVelocityProjectionOnWheelDirectionMPS / wheelRadiusMeters};

        if (skidding) {
            const wpi::units::radians_per_second_t skiddingEquilibriumWheelSpeed = config.driveMotorConfigs.CalculateMechanismVelocity(
                config.driveMotorConfigs.CalculateCurrent(wpi::units::newton_meter_t{propellingForceNewtons * wheelRadiusMeters}), driveMotorAppliedVoltage_);
            driveWheelFinalSpeed_ = driveWheelFinalSpeed_ * 0.5 + skiddingEquilibriumWheelSpeed * 0.5;
        }

        return physics::Force2d::FromPolar(wpi::units::newton_t{propellingForceNewtons}, moduleWorldFacing);
    }

    wpi::units::newton_meter_t SwerveModuleSimulation::GetDriveWheelTorque() {
        driveMotorAppliedVoltage_ = driveMotorController_->UpdateControlSignal(driveWheelFinalPosition_, driveWheelFinalSpeed_,
                                                                               GetDriveEncoderUnGearedPosition(), GetDriveEncoderUnGearedSpeed());
        driveMotorAppliedVoltage_ = motorsims::SimulatedBattery::Clamp(driveMotorAppliedVoltage_);
        driveMotorStatorCurrent_ = config.driveMotorConfigs.CalculateCurrent(driveWheelFinalSpeed_, driveMotorAppliedVoltage_);
        const wpi::units::newton_meter_t driveWheelTorque = config.driveMotorConfigs.CalculateTorque(driveMotorStatorCurrent_);
        return wpi::units::newton_meter_t{
            wpi::math::ApplyDeadband(driveWheelTorque.value(), config.driveMotorConfigs.friction.value(), std::numeric_limits<double>::infinity())};
    }

    wpi::math::SwerveModuleVelocity SwerveModuleSimulation::GetCurrentState() const {
        return wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{GetDriveWheelFinalSpeed().value() * config.wheelRadius.value()},
                                               GetSteerAbsoluteFacing()};
    }

    wpi::math::SwerveModuleVelocity SwerveModuleSimulation::GetFreeSpinState() const {
        const wpi::units::radians_per_second_t freeSpinWheelSpeed = config.driveMotorConfigs.CalculateMechanismVelocity(
            config.driveMotorConfigs.CalculateCurrent(config.driveMotorConfigs.friction), driveMotorAppliedVoltage_);
        return wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t{freeSpinWheelSpeed.value() * config.wheelRadius.value()},
                                               GetSteerAbsoluteFacing()};
    }

    void SwerveModuleSimulation::UpdateEncoderCaches(double driveWheelDistanceScale) {
        driveWheelFinalPosition_ = driveWheelFinalPosition_ + driveWheelFinalSpeed_ * SimulatedArena::GetSimulationDt() * driveWheelDistanceScale;
        steerAbsoluteAngleCache_.pop_front();
        steerAbsoluteAngleCache_.push_back(GetSteerAbsoluteAngle());
        driveWheelFinalPositionCache_.pop_front();
        driveWheelFinalPositionCache_.push_back(driveWheelFinalPosition_);
    }

    void SwerveModuleSimulation::SetDriveWheelOdometryDistanceScale(double driveWheelOdometryDistanceScale) {
        driveWheelOdometryDistanceScale_ = SanitizeDriveWheelOdometryScale(driveWheelOdometryDistanceScale);
    }

    wpi::units::ampere_t SwerveModuleSimulation::GetDriveMotorSupplyCurrent() const {
        return GetDriveMotorStatorCurrent() * (driveMotorAppliedVoltage_ / motorsims::SimulatedBattery::GetBatteryVoltage());
    }

    wpi::units::radian_t SwerveModuleSimulation::GetSteerRelativeEncoderPosition() const {
        return GetSteerAbsoluteAngle() * config.steerGearRatio + steerRelativeEncoderOffSet_;
    }

    std::vector<wpi::units::radian_t> SwerveModuleSimulation::GetCachedDriveEncoderUnGearedPositions() const {
        std::vector<wpi::units::radian_t> positions;
        positions.reserve(driveWheelFinalPositionCache_.size());
        for (const wpi::units::radian_t driveWheelFinalPosition : driveWheelFinalPositionCache_)
            positions.push_back(driveWheelFinalPosition * config.driveGearRatio);
        return positions;
    }

    std::vector<wpi::units::radian_t> SwerveModuleSimulation::GetCachedDriveWheelFinalPositions() const {
        return {driveWheelFinalPositionCache_.begin(), driveWheelFinalPositionCache_.end()};
    }

    std::vector<wpi::units::radian_t> SwerveModuleSimulation::GetCachedSteerRelativeEncoderPositions() const {
        std::vector<wpi::units::radian_t> positions;
        positions.reserve(steerAbsoluteAngleCache_.size());
        for (const wpi::units::radian_t absoluteAngle : steerAbsoluteAngleCache_)
            positions.push_back(absoluteAngle * config.steerGearRatio + steerRelativeEncoderOffSet_);
        return positions;
    }

    std::vector<wpi::math::Rotation2d> SwerveModuleSimulation::GetCachedSteerAbsolutePositions() const {
        std::vector<wpi::math::Rotation2d> positions;
        positions.reserve(steerAbsoluteAngleCache_.size());
        for (const wpi::units::radian_t absoluteAngle : steerAbsoluteAngleCache_)
            positions.emplace_back(absoluteAngle);
        return positions;
    }
} // namespace maplesim::simulation::drivesims
