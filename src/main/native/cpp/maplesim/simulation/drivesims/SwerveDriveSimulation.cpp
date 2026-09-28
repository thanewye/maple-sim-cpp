#include "pch.h"

#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <wpi/array.h>

#include "maplesim/physics/Vector2d.h"
#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/utils/mathutils/GeometryConvertor.h"
#include "maplesim/utils/mathutils/MapleCommonMath.h"

namespace maplesim::simulation::drivesims {
    namespace {
        using utils::mathutils::GeometryConvertor::GetChassisSpeedsTranslationalComponent;
        using utils::mathutils::MapleCommonMath::GetAngle;

        [[nodiscard]] SwerveDriveSimulation::SwerveModuleSimulations
        CreateModuleSimulations(const std::array<configs::SwerveModuleSimulationFactory, configs::kSwerveModuleCount>& swerveModuleSimulationFactories) {
            SwerveDriveSimulation::SwerveModuleSimulations moduleSimulations;
            for (int i = 0; i < configs::kSwerveModuleCount; i++)
                moduleSimulations[i] = swerveModuleSimulationFactories[i]();
            return moduleSimulations;
        }
    } // namespace

    SwerveDriveSimulation::SwerveDriveSimulation(configs::DriveTrainSimulationConfig config, const frc::Pose2d& initialPoseOnField)
        : AbstractDriveTrainSimulation(std::move(config), initialPoseOnField)
        , moduleTranslations_(this->config.moduleTranslations)
        , gyroSimulation_(this->config.gyroSimulationFactory())
        , kinematics_(moduleTranslations_[0], moduleTranslations_[1], moduleTranslations_[2], moduleTranslations_[3])
        , moduleSimulations_(CreateModuleSimulations(this->config.swerveModuleSimulationFactories))
        , gravityForceOnEachModule_(this->config.robotMass.value() * 9.8 / configs::kSwerveModuleCount) {
        SetLinearDamping(1.4);
        SetAngularDamping(1.4);
    }

    void SwerveDriveSimulation::SimulationSubTick() {
        SimulateChassisFrictionForce();
        SimulateChassisFrictionTorque();
        SimulateModulePropellingForces();
        gyroSimulation_->UpdateSimulationSubTick(GetAngularVelocity());
    }

    void SwerveDriveSimulation::SimulateChassisFrictionForce() {
        const frc::ChassisSpeeds moduleSpeeds = GetModuleSpeeds();
        const frc::ChassisSpeeds differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative = moduleSpeeds - GetDriveTrainSimulatedChassisSpeedsRobotRelative();
        const frc::Translation2d floorAndModuleSpeedsDiffFieldRelative =
            GetChassisSpeedsTranslationalComponent(differenceBetweenFloorSpeedAndModuleSpeedsRobotRelative).RotateBy(GetSimulatedDriveTrainPose().Rotation());

        constexpr double kFrictionForceGain = 3.0;
        const double totalGrippingForceNewtons =
            moduleSimulations_[0]->config.GetGrippingForce(gravityForceOnEachModule_).value() * configs::kSwerveModuleCount;
        const physics::Force2d speedsDifferenceFrictionForce = physics::Force2d::FromPolar(
            units::newton_t{
                std::min(kFrictionForceGain * totalGrippingForceNewtons * floorAndModuleSpeedsDiffFieldRelative.Norm().value(), totalGrippingForceNewtons)},
            GetAngle(floorAndModuleSpeedsDiffFieldRelative));

        const frc::ChassisSpeeds moduleSpeedsFieldRelative = frc::ChassisSpeeds::FromRobotRelativeSpeeds(moduleSpeeds, GetSimulatedDriveTrainPose().Rotation());
        const frc::Rotation2d dTheta =
            GetAngle(GetChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative)) - GetAngle(previousModuleSpeedsFieldRelative_);
        const double orbitalAngularVelocity = dTheta.Radians().value() / SimulatedArena::GetSimulationDt().value();
        const frc::Rotation2d centripetalForceDirection = GetAngle(previousModuleSpeedsFieldRelative_) + frc::Rotation2d{units::degree_t{90}};
        const physics::Force2d centripetalFrictionForce = physics::Force2d::FromPolar(
            units::newton_t{previousModuleSpeedsFieldRelative_.Norm().value() * orbitalAngularVelocity * config.robotMass.value()}, centripetalForceDirection);
        previousModuleSpeedsFieldRelative_ = GetChassisSpeedsTranslationalComponent(moduleSpeedsFieldRelative);

        const physics::Force2d totalFrictionForceUnlimited = centripetalFrictionForce + speedsDifferenceFrictionForce;
        const physics::Force2d totalFrictionForce = physics::Force2d::FromPolar(
            units::newton_t{std::min(totalGrippingForceNewtons, totalFrictionForceUnlimited.Norm().value())}, totalFrictionForceUnlimited.Angle());
        ApplyForce(totalFrictionForce);
    }

    void SwerveDriveSimulation::SimulateChassisFrictionTorque() {
        const double maxAngularVelocityRadPerSec = MaxAngularVelocity().value();
        const double desiredRotationalMotionPercent = std::abs(GetDesiredSpeed().omega.value() / maxAngularVelocityRadPerSec);
        const double actualRotationalMotionPercent = std::abs(GetAngularVelocity().value() / maxAngularVelocityRadPerSec);
        const double differenceBetweenFloorSpeedAndModuleSpeed = GetModuleSpeeds().omega.value() - GetAngularVelocity().value();
        const double grippingTorqueMagnitude = moduleSimulations_[0]->config.GetGrippingForce(gravityForceOnEachModule_).value() *
                                               moduleTranslations_[0].Norm().value() * configs::kSwerveModuleCount;
        constexpr double kFrictionTorqueGain = 1;

        if (actualRotationalMotionPercent < 0.01 && desiredRotationalMotionPercent < 0.02) SetAngularVelocity(units::radians_per_second_t{0.0});
        else
            ApplyTorque(units::newton_meter_t{std::copysign(
                std::min(kFrictionTorqueGain * grippingTorqueMagnitude * std::abs(differenceBetweenFloorSpeedAndModuleSpeed), grippingTorqueMagnitude),
                differenceBetweenFloorSpeedAndModuleSpeed)});
    }

    void SwerveDriveSimulation::SimulateModulePropellingForces() {
        for (int i = 0; i < configs::kSwerveModuleCount; i++) {
            const frc::Translation2d moduleWorldPosition = GetWorldPoint(moduleTranslations_[i]);
            const physics::Force2d moduleForce = moduleSimulations_[i]->UpdateSimulationSubTickGetModuleForce(
                GetVelocityAtPoint(moduleWorldPosition), GetSimulatedDriveTrainPose().Rotation(), gravityForceOnEachModule_);
            ApplyForce(moduleForce, moduleWorldPosition);
        }
    }

    frc::ChassisSpeeds SwerveDriveSimulation::GetDesiredSpeed() const {
        return kinematics_.ToChassisSpeeds(wpi::array<frc::SwerveModuleState, configs::kSwerveModuleCount>{
            moduleSimulations_[0]->GetFreeSpinState(), moduleSimulations_[1]->GetFreeSpinState(), moduleSimulations_[2]->GetFreeSpinState(),
            moduleSimulations_[3]->GetFreeSpinState()});
    }

    frc::ChassisSpeeds SwerveDriveSimulation::GetModuleSpeeds() const {
        return kinematics_.ToChassisSpeeds(wpi::array<frc::SwerveModuleState, configs::kSwerveModuleCount>{
            moduleSimulations_[0]->GetCurrentState(), moduleSimulations_[1]->GetCurrentState(), moduleSimulations_[2]->GetCurrentState(),
            moduleSimulations_[3]->GetCurrentState()});
    }

    units::meters_per_second_t SwerveDriveSimulation::MaxLinearVelocity() const {
        return moduleSimulations_[0]->config.MaximumGroundSpeed();
    }

    units::meters_per_second_squared_t SwerveDriveSimulation::MaxLinearAcceleration(units::ampere_t statorCurrentLimit) const {
        return moduleSimulations_[0]->config.MaxAcceleration(config.robotMass, configs::kSwerveModuleCount, statorCurrentLimit);
    }

    units::radians_per_second_t SwerveDriveSimulation::MaxAngularVelocity() const {
        return units::radians_per_second_t{MaxLinearVelocity().value() / config.DriveBaseRadius().value()};
    }

    units::radians_per_second_squared_t SwerveDriveSimulation::MaxAngularAcceleration(units::ampere_t statorCurrentLimit) const {
        return units::radians_per_second_squared_t{
            moduleSimulations_[0]->config.GetTheoreticalPropellingForcePerModule(config.robotMass, configs::kSwerveModuleCount, statorCurrentLimit).value() *
            moduleTranslations_[0].Norm().value() * configs::kSwerveModuleCount / GetMomentOfInertia().value()};
    }
} // namespace maplesim::simulation::drivesims
