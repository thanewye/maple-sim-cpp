#include "pch.h"

#include "maplesim/simulation/drivesims/SelfControlledSwerveDriveSimulation.h"

#include <array>
#include <cstddef>
#include <numbers>

#include <wpi/system/Timer.hpp>

#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/utils/mathutils/SwerveStateProjection.h"

namespace maplesim::simulation::drivesims {
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::SelfControlledModuleSimulation(SwerveModuleSimulation& moduleSimulation)
        : moduleSimulation_(moduleSimulation)
        , driveMotor_(moduleSimulation_.UseGenericMotorControllerForDrive())
        , steerMotor_(moduleSimulation_.UseGenericControllerForSteer()) {
        driveMotor_.WithCurrentLimit(driveCurrentLimit_);
        steerController_.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    }

    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation&
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::WithSteerPID(const wpi::math::PIDController& steerController) {
        steerController_.SetPID(steerController.GetP(), steerController.GetI(), steerController.GetD());
        steerController_.Reset();
        steerController_.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
        return *this;
    }

    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation&
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::WithCurrentLimits(wpi::units::ampere_t driveCurrentLimit,
                                                                                           wpi::units::ampere_t steerCurrentLimit) {
        driveCurrentLimit_ = driveCurrentLimit;
        driveMotor_.WithCurrentLimit(driveCurrentLimit);
        steerMotor_.WithCurrentLimit(steerCurrentLimit);
        return *this;
    }

    wpi::math::SwerveModuleVelocity
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::OptimizeAndRunModuleState(wpi::math::SwerveModuleVelocity setPoint) {
        setPoint = setPoint.Optimize(moduleSimulation_.GetSteerAbsoluteFacing());
        RunModuleState(setPoint);
        return setPoint;
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunModuleState(const wpi::math::SwerveModuleVelocity& setPoint) {
        const wpi::units::meters_per_second_t cosineProjectedSpeed =
            utils::mathutils::SwerveStateProjection::Project(setPoint, moduleSimulation_.GetSteerAbsoluteFacing());
        const wpi::units::radians_per_second_t driveWheelVelocitySetPoint{cosineProjectedSpeed.value() / moduleSimulation_.config.wheelRadius.value()};

        driveMotor_.RequestVoltage(moduleSimulation_.config.driveMotorConfigs.CalculateVoltage(wpi::units::ampere_t{0.0}, driveWheelVelocitySetPoint));
        steerMotor_.RequestVoltage(
            wpi::units::volt_t{steerController_.Calculate(moduleSimulation_.GetSteerAbsoluteFacing().Radians().value(), setPoint.angle.Radians().value())});
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunDriveMotorCharacterization(const wpi::math::Rotation2d& desiredModuleFacing,
                                                                                                            wpi::units::volt_t voltage) {
        driveMotor_.RequestVoltage(voltage);
        steerMotor_.RequestVoltage(wpi::units::volt_t{
            steerController_.Calculate(moduleSimulation_.GetSteerAbsoluteFacing().Radians().value(), desiredModuleFacing.Radians().value())});
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunSteerMotorCharacterization(wpi::units::volt_t voltage) {
        driveMotor_.RequestVoltage(wpi::units::volt_t{0.0});
        steerMotor_.RequestVoltage(voltage);
    }

    wpi::math::SwerveModuleVelocity SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::GetMeasuredState() const {
        return moduleSimulation_.GetCurrentState();
    }

    wpi::math::SwerveModulePosition SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::GetModulePosition() const {
        return wpi::math::SwerveModulePosition{
            wpi::units::meter_t{moduleSimulation_.GetDriveWheelFinalPosition().value() * moduleSimulation_.config.wheelRadius.value()},
            moduleSimulation_.GetSteerAbsoluteFacing()};
    }

    SelfControlledSwerveDriveSimulation::SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation)
        : SelfControlledSwerveDriveSimulation(swerveDriveSimulation, wpi::util::array<double, 3>{0.1, 0.1, 0.1}, wpi::util::array<double, 3>{0.9, 0.9, 0.9}) {}

    SelfControlledSwerveDriveSimulation::SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation,
                                                                             const wpi::util::array<double, 3>& stateStdDevs,
                                                                             const wpi::util::array<double, 3>& visionMeasurementStdDevs)
        : swerveDriveSimulation_(swerveDriveSimulation)
        , moduleSimulations_{SelfControlledModuleSimulation{*swerveDriveSimulation_.GetModules()[0]},
                             SelfControlledModuleSimulation{*swerveDriveSimulation_.GetModules()[1]},
                             SelfControlledModuleSimulation{*swerveDriveSimulation_.GetModules()[2]},
                             SelfControlledModuleSimulation{*swerveDriveSimulation_.GetModules()[3]}}
        , kinematics_(swerveDriveSimulation_.config.moduleTranslations[0], swerveDriveSimulation_.config.moduleTranslations[1],
                      swerveDriveSimulation_.config.moduleTranslations[2], swerveDriveSimulation_.config.moduleTranslations[3])
        , poseEstimator_(kinematics_, GetRawGyroAngle(), GetLatestModulePositions(), GetActualPoseInSimulationWorld(), stateStdDevs, visionMeasurementStdDevs) {
    }

    void SelfControlledSwerveDriveSimulation::Periodic() {
        const CachedModulePositions cachedModulePositions = GetCachedModulePositions();
        const std::vector<wpi::math::Rotation2d> cachedGyroReadings = swerveDriveSimulation_.GetGyroSimulation().GetCachedGyroReadings();
        const int subTickCount = SimulatedArena::GetSimulationSubTicksIn1Period();
        const wpi::units::second_t now = wpi::Timer::GetTimestamp();
        const wpi::units::second_t dt = SimulatedArena::GetSimulationDt();

        for (int index = 0; index < subTickCount; index++) {
            const wpi::units::second_t timestamp = now - dt * static_cast<double>(subTickCount - 1 - index);
            poseEstimator_.UpdateWithTime(timestamp, cachedGyroReadings[index], cachedModulePositions[index]);
        }
    }

    SelfControlledSwerveDriveSimulation::ModulePositions SelfControlledSwerveDriveSimulation::GetLatestModulePositions() const {
        return ModulePositions{moduleSimulations_[0].GetModulePosition(), moduleSimulations_[1].GetModulePosition(), moduleSimulations_[2].GetModulePosition(),
                               moduleSimulations_[3].GetModulePosition()};
    }

    SelfControlledSwerveDriveSimulation::CachedModulePositions SelfControlledSwerveDriveSimulation::GetCachedModulePositions() const {
        const int subTickCount = SimulatedArena::GetSimulationSubTicksIn1Period();
        CachedModulePositions cachedModulePositions(static_cast<std::size_t>(subTickCount), ModulePositions{wpi::util::empty_array});
        const wpi::units::meter_t cachedWheelRadius = moduleSimulations_[0].GetModuleSimulation().config.wheelRadius;

        for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; moduleIndex++) {
            const SwerveModuleSimulation& moduleSimulation = moduleSimulations_[moduleIndex].GetModuleSimulation();
            const std::vector<wpi::units::radian_t> wheelPositions = moduleSimulation.GetCachedDriveWheelFinalPositions();
            const std::vector<wpi::math::Rotation2d> moduleFacings = moduleSimulation.GetCachedSteerAbsolutePositions();

            for (int timestampIndex = 0; timestampIndex < subTickCount; timestampIndex++) {
                cachedModulePositions[timestampIndex][moduleIndex] = wpi::math::SwerveModulePosition{
                    wpi::units::meter_t{wheelPositions[timestampIndex].value() * cachedWheelRadius.value()}, moduleFacings[timestampIndex]};
            }
        }

        return cachedModulePositions;
    }

    wpi::math::Rotation2d SelfControlledSwerveDriveSimulation::GetRawGyroAngle() const {
        return swerveDriveSimulation_.GetGyroSimulation().GetGyroReading();
    }

    wpi::math::Pose2d SelfControlledSwerveDriveSimulation::GetOdometryEstimatedPose() const {
        return poseEstimator_.GetEstimatedPosition();
    }

    void SelfControlledSwerveDriveSimulation::ResetOdometry(const wpi::math::Pose2d& pose) {
        poseEstimator_.ResetPosition(GetRawGyroAngle(), GetLatestModulePositions(), pose);
    }

    void SelfControlledSwerveDriveSimulation::AddVisionEstimation(const wpi::math::Pose2d& robotPoseMeters, wpi::units::second_t timestamp) {
        poseEstimator_.AddVisionMeasurement(robotPoseMeters, timestamp);
    }

    void SelfControlledSwerveDriveSimulation::AddVisionEstimation(const wpi::math::Pose2d& robotPoseMeters, wpi::units::second_t timestamp,
                                                                  const wpi::util::array<double, 3>& measurementStdDevs) {
        poseEstimator_.AddVisionMeasurement(robotPoseMeters, timestamp, measurementStdDevs);
    }

    void SelfControlledSwerveDriveSimulation::RunChassisSpeeds(wpi::math::ChassisVelocities chassisSpeeds,
                                                               const wpi::math::Translation2d& centerOfRotationMeters, bool fieldCentricDrive,
                                                               bool discretizeSpeeds) {
        if (fieldCentricDrive) chassisSpeeds = chassisSpeeds.ToRobotRelative(GetOdometryEstimatedPose().Rotation());
        if (discretizeSpeeds) {
            const wpi::units::second_t robotPeriod = SimulatedArena::GetSimulationDt() * static_cast<double>(SimulatedArena::GetSimulationSubTicksIn1Period());
            chassisSpeeds = chassisSpeeds.Discretize(robotPeriod);
        }
        RunSwerveStates(kinematics_.ToSwerveModuleVelocities(chassisSpeeds, centerOfRotationMeters));
    }

    void SelfControlledSwerveDriveSimulation::RunSwerveStates(const ModuleStates& setPoints) {
        for (std::size_t index = 0; index < kModuleCount; index++)
            setPointsOptimized_[index] = moduleSimulations_[index].OptimizeAndRunModuleState(setPoints[index]);
    }

    SelfControlledSwerveDriveSimulation::ModuleStates SelfControlledSwerveDriveSimulation::GetMeasuredStates() const {
        return ModuleStates{moduleSimulations_[0].GetMeasuredState(), moduleSimulations_[1].GetMeasuredState(), moduleSimulations_[2].GetMeasuredState(),
                            moduleSimulations_[3].GetMeasuredState()};
    }

    wpi::math::ChassisVelocities SelfControlledSwerveDriveSimulation::GetMeasuredSpeedsFieldRelative(bool useGyroForAngularVelocity) const {
        return GetMeasuredSpeedsRobotRelative(useGyroForAngularVelocity).ToFieldRelative(GetOdometryEstimatedPose().Rotation());
    }

    wpi::math::ChassisVelocities SelfControlledSwerveDriveSimulation::GetMeasuredSpeedsRobotRelative(bool useGyroForAngularVelocity) const {
        const wpi::math::ChassisVelocities swerveSpeeds = kinematics_.ToChassisVelocities(GetMeasuredStates());
        return wpi::math::ChassisVelocities{swerveSpeeds.vx, swerveSpeeds.vy,
                                            useGyroForAngularVelocity ? swerveDriveSimulation_.GetGyroSimulation().GetMeasuredAngularVelocity()
                                                                      : swerveSpeeds.omega};
    }

    wpi::math::Pose2d SelfControlledSwerveDriveSimulation::GetActualPoseInSimulationWorld() const {
        return swerveDriveSimulation_.GetSimulatedDriveTrainPose();
    }

    wpi::math::ChassisVelocities SelfControlledSwerveDriveSimulation::GetActualSpeedsFieldRelative() const {
        return swerveDriveSimulation_.GetDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    wpi::math::ChassisVelocities SelfControlledSwerveDriveSimulation::GetActualSpeedsRobotRelative() const {
        return swerveDriveSimulation_.GetDriveTrainSimulatedChassisSpeedsRobotRelative();
    }

    void SelfControlledSwerveDriveSimulation::SetSimulationWorldPose(const wpi::math::Pose2d& robotPose) {
        swerveDriveSimulation_.SetSimulationWorldPose(robotPose);
    }

    SelfControlledSwerveDriveSimulation& SelfControlledSwerveDriveSimulation::WithSteerPID(const wpi::math::PIDController& steerController) {
        for (SelfControlledModuleSimulation& moduleSimulation : moduleSimulations_)
            moduleSimulation.WithSteerPID(steerController);
        return *this;
    }

    SelfControlledSwerveDriveSimulation& SelfControlledSwerveDriveSimulation::WithCurrentLimits(wpi::units::ampere_t driveCurrentLimit,
                                                                                                wpi::units::ampere_t steerCurrentLimit) {
        for (SelfControlledModuleSimulation& moduleSimulation : moduleSimulations_)
            moduleSimulation.WithCurrentLimits(driveCurrentLimit, steerCurrentLimit);
        return *this;
    }

    wpi::units::meters_per_second_t SelfControlledSwerveDriveSimulation::MaxLinearVelocity() const {
        return swerveDriveSimulation_.MaxLinearVelocity();
    }

    wpi::units::meters_per_second_squared_t SelfControlledSwerveDriveSimulation::MaxLinearAcceleration() const {
        return swerveDriveSimulation_.MaxLinearAcceleration(moduleSimulations_[0].GetDriveCurrentLimit());
    }

    wpi::units::meter_t SelfControlledSwerveDriveSimulation::TrackWidthY() const {
        return swerveDriveSimulation_.config.TrackWidthY();
    }

    wpi::units::meter_t SelfControlledSwerveDriveSimulation::TrackLengthX() const {
        return swerveDriveSimulation_.config.TrackLengthX();
    }

    wpi::units::meter_t SelfControlledSwerveDriveSimulation::DriveBaseRadius() const {
        return swerveDriveSimulation_.DriveBaseRadius();
    }

    wpi::units::radians_per_second_t SelfControlledSwerveDriveSimulation::MaxAngularVelocity() const {
        return swerveDriveSimulation_.MaxAngularVelocity();
    }

    wpi::units::radians_per_second_squared_t SelfControlledSwerveDriveSimulation::MaxAngularAcceleration() const {
        return swerveDriveSimulation_.MaxAngularAcceleration(moduleSimulations_[0].GetDriveCurrentLimit());
    }
} // namespace maplesim::simulation::drivesims
