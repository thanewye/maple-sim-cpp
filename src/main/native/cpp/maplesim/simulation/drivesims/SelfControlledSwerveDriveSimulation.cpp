#include "pch.h"

#include "maplesim/simulation/drivesims/SelfControlledSwerveDriveSimulation.h"

#include <array>
#include <cstddef>
#include <numbers>

#include <frc/Timer.h>

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
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::WithSteerPID(const frc::PIDController& steerController) {
        steerController_.SetPID(steerController.GetP(), steerController.GetI(), steerController.GetD());
        steerController_.Reset();
        steerController_.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
        return *this;
    }

    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation&
    SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::WithCurrentLimits(units::ampere_t driveCurrentLimit,
                                                                                           units::ampere_t steerCurrentLimit) {
        driveCurrentLimit_ = driveCurrentLimit;
        driveMotor_.WithCurrentLimit(driveCurrentLimit);
        steerMotor_.WithCurrentLimit(steerCurrentLimit);
        return *this;
    }

    frc::SwerveModuleState SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::OptimizeAndRunModuleState(frc::SwerveModuleState setPoint) {
        setPoint.Optimize(moduleSimulation_.GetSteerAbsoluteFacing());
        RunModuleState(setPoint);
        return setPoint;
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunModuleState(const frc::SwerveModuleState& setPoint) {
        const units::meters_per_second_t cosineProjectedSpeed =
            utils::mathutils::SwerveStateProjection::Project(setPoint, moduleSimulation_.GetSteerAbsoluteFacing());
        const units::radians_per_second_t driveWheelVelocitySetPoint{cosineProjectedSpeed.value() / moduleSimulation_.config.wheelRadius.value()};

        driveMotor_.RequestVoltage(moduleSimulation_.config.driveMotorConfigs.CalculateVoltage(units::ampere_t{0.0}, driveWheelVelocitySetPoint));
        steerMotor_.RequestVoltage(
            units::volt_t{steerController_.Calculate(moduleSimulation_.GetSteerAbsoluteFacing().Radians().value(), setPoint.angle.Radians().value())});
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunDriveMotorCharacterization(const frc::Rotation2d& desiredModuleFacing,
                                                                                                            units::volt_t voltage) {
        driveMotor_.RequestVoltage(voltage);
        steerMotor_.RequestVoltage(
            units::volt_t{steerController_.Calculate(moduleSimulation_.GetSteerAbsoluteFacing().Radians().value(), desiredModuleFacing.Radians().value())});
    }

    void SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::RunSteerMotorCharacterization(units::volt_t voltage) {
        driveMotor_.RequestVoltage(units::volt_t{0.0});
        steerMotor_.RequestVoltage(voltage);
    }

    frc::SwerveModuleState SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::GetMeasuredState() const {
        return moduleSimulation_.GetCurrentState();
    }

    frc::SwerveModulePosition SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation::GetModulePosition() const {
        return frc::SwerveModulePosition{units::meter_t{moduleSimulation_.GetDriveWheelFinalPosition().value() * moduleSimulation_.config.wheelRadius.value()},
                                         moduleSimulation_.GetSteerAbsoluteFacing()};
    }

    SelfControlledSwerveDriveSimulation::SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation)
        : SelfControlledSwerveDriveSimulation(swerveDriveSimulation, wpi::array<double, 3>{0.1, 0.1, 0.1}, wpi::array<double, 3>{0.9, 0.9, 0.9}) {}

    SelfControlledSwerveDriveSimulation::SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation,
                                                                             const wpi::array<double, 3>& stateStdDevs,
                                                                             const wpi::array<double, 3>& visionMeasurementStdDevs)
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
        const std::vector<frc::Rotation2d> cachedGyroReadings = swerveDriveSimulation_.GetGyroSimulation().GetCachedGyroReadings();
        const int subTickCount = SimulatedArena::GetSimulationSubTicksIn1Period();
        const units::second_t now = frc::Timer::GetFPGATimestamp();
        const units::second_t dt = SimulatedArena::GetSimulationDt();

        for (int index = 0; index < subTickCount; index++) {
            const units::second_t timestamp = now - dt * static_cast<double>(subTickCount - 1 - index);
            poseEstimator_.UpdateWithTime(timestamp, cachedGyroReadings[index], cachedModulePositions[index]);
        }
    }

    SelfControlledSwerveDriveSimulation::ModulePositions SelfControlledSwerveDriveSimulation::GetLatestModulePositions() const {
        return ModulePositions{moduleSimulations_[0].GetModulePosition(), moduleSimulations_[1].GetModulePosition(), moduleSimulations_[2].GetModulePosition(),
                               moduleSimulations_[3].GetModulePosition()};
    }

    SelfControlledSwerveDriveSimulation::CachedModulePositions SelfControlledSwerveDriveSimulation::GetCachedModulePositions() const {
        const int subTickCount = SimulatedArena::GetSimulationSubTicksIn1Period();
        CachedModulePositions cachedModulePositions(static_cast<std::size_t>(subTickCount), ModulePositions{wpi::empty_array});
        const units::meter_t cachedWheelRadius = moduleSimulations_[0].GetModuleSimulation().config.wheelRadius;

        for (std::size_t moduleIndex = 0; moduleIndex < kModuleCount; moduleIndex++) {
            const SwerveModuleSimulation& moduleSimulation = moduleSimulations_[moduleIndex].GetModuleSimulation();
            const std::vector<units::radian_t> wheelPositions = moduleSimulation.GetCachedDriveWheelFinalPositions();
            const std::vector<frc::Rotation2d> moduleFacings = moduleSimulation.GetCachedSteerAbsolutePositions();

            for (int timestampIndex = 0; timestampIndex < subTickCount; timestampIndex++) {
                cachedModulePositions[timestampIndex][moduleIndex] = frc::SwerveModulePosition{
                    units::meter_t{wheelPositions[timestampIndex].value() * cachedWheelRadius.value()}, moduleFacings[timestampIndex]};
            }
        }

        return cachedModulePositions;
    }

    frc::Rotation2d SelfControlledSwerveDriveSimulation::GetRawGyroAngle() const {
        return swerveDriveSimulation_.GetGyroSimulation().GetGyroReading();
    }

    frc::Pose2d SelfControlledSwerveDriveSimulation::GetOdometryEstimatedPose() const {
        return poseEstimator_.GetEstimatedPosition();
    }

    void SelfControlledSwerveDriveSimulation::ResetOdometry(const frc::Pose2d& pose) {
        poseEstimator_.ResetPosition(GetRawGyroAngle(), GetLatestModulePositions(), pose);
    }

    void SelfControlledSwerveDriveSimulation::AddVisionEstimation(const frc::Pose2d& robotPoseMeters, units::second_t timestamp) {
        poseEstimator_.AddVisionMeasurement(robotPoseMeters, timestamp);
    }

    void SelfControlledSwerveDriveSimulation::AddVisionEstimation(const frc::Pose2d& robotPoseMeters, units::second_t timestamp,
                                                                  const wpi::array<double, 3>& measurementStdDevs) {
        poseEstimator_.AddVisionMeasurement(robotPoseMeters, timestamp, measurementStdDevs);
    }

    void SelfControlledSwerveDriveSimulation::RunChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, const frc::Translation2d& centerOfRotationMeters,
                                                               bool fieldCentricDrive, bool discretizeSpeeds) {
        if (fieldCentricDrive) chassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(chassisSpeeds, GetOdometryEstimatedPose().Rotation());
        if (discretizeSpeeds) {
            const units::second_t robotPeriod = SimulatedArena::GetSimulationDt() * static_cast<double>(SimulatedArena::GetSimulationSubTicksIn1Period());
            chassisSpeeds = frc::ChassisSpeeds::Discretize(chassisSpeeds, robotPeriod);
        }
        RunSwerveStates(kinematics_.ToSwerveModuleStates(chassisSpeeds, centerOfRotationMeters));
    }

    void SelfControlledSwerveDriveSimulation::RunSwerveStates(const ModuleStates& setPoints) {
        for (std::size_t index = 0; index < kModuleCount; index++)
            setPointsOptimized_[index] = moduleSimulations_[index].OptimizeAndRunModuleState(setPoints[index]);
    }

    SelfControlledSwerveDriveSimulation::ModuleStates SelfControlledSwerveDriveSimulation::GetMeasuredStates() const {
        return ModuleStates{moduleSimulations_[0].GetMeasuredState(), moduleSimulations_[1].GetMeasuredState(), moduleSimulations_[2].GetMeasuredState(),
                            moduleSimulations_[3].GetMeasuredState()};
    }

    frc::ChassisSpeeds SelfControlledSwerveDriveSimulation::GetMeasuredSpeedsFieldRelative(bool useGyroForAngularVelocity) const {
        return frc::ChassisSpeeds::FromRobotRelativeSpeeds(GetMeasuredSpeedsRobotRelative(useGyroForAngularVelocity), GetOdometryEstimatedPose().Rotation());
    }

    frc::ChassisSpeeds SelfControlledSwerveDriveSimulation::GetMeasuredSpeedsRobotRelative(bool useGyroForAngularVelocity) const {
        const frc::ChassisSpeeds swerveSpeeds = kinematics_.ToChassisSpeeds(GetMeasuredStates());
        return frc::ChassisSpeeds{swerveSpeeds.vx, swerveSpeeds.vy,
                                  useGyroForAngularVelocity ? swerveDriveSimulation_.GetGyroSimulation().GetMeasuredAngularVelocity() : swerveSpeeds.omega};
    }

    frc::Pose2d SelfControlledSwerveDriveSimulation::GetActualPoseInSimulationWorld() const {
        return swerveDriveSimulation_.GetSimulatedDriveTrainPose();
    }

    frc::ChassisSpeeds SelfControlledSwerveDriveSimulation::GetActualSpeedsFieldRelative() const {
        return swerveDriveSimulation_.GetDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    frc::ChassisSpeeds SelfControlledSwerveDriveSimulation::GetActualSpeedsRobotRelative() const {
        return swerveDriveSimulation_.GetDriveTrainSimulatedChassisSpeedsRobotRelative();
    }

    void SelfControlledSwerveDriveSimulation::SetSimulationWorldPose(const frc::Pose2d& robotPose) {
        swerveDriveSimulation_.SetSimulationWorldPose(robotPose);
    }

    SelfControlledSwerveDriveSimulation& SelfControlledSwerveDriveSimulation::WithSteerPID(const frc::PIDController& steerController) {
        for (SelfControlledModuleSimulation& moduleSimulation : moduleSimulations_)
            moduleSimulation.WithSteerPID(steerController);
        return *this;
    }

    SelfControlledSwerveDriveSimulation& SelfControlledSwerveDriveSimulation::WithCurrentLimits(units::ampere_t driveCurrentLimit,
                                                                                                units::ampere_t steerCurrentLimit) {
        for (SelfControlledModuleSimulation& moduleSimulation : moduleSimulations_)
            moduleSimulation.WithCurrentLimits(driveCurrentLimit, steerCurrentLimit);
        return *this;
    }

    units::meters_per_second_t SelfControlledSwerveDriveSimulation::MaxLinearVelocity() const {
        return swerveDriveSimulation_.MaxLinearVelocity();
    }

    units::meters_per_second_squared_t SelfControlledSwerveDriveSimulation::MaxLinearAcceleration() const {
        return swerveDriveSimulation_.MaxLinearAcceleration(moduleSimulations_[0].GetDriveCurrentLimit());
    }

    units::meter_t SelfControlledSwerveDriveSimulation::TrackWidthY() const {
        return swerveDriveSimulation_.config.TrackWidthY();
    }

    units::meter_t SelfControlledSwerveDriveSimulation::TrackLengthX() const {
        return swerveDriveSimulation_.config.TrackLengthX();
    }

    units::meter_t SelfControlledSwerveDriveSimulation::DriveBaseRadius() const {
        return swerveDriveSimulation_.DriveBaseRadius();
    }

    units::radians_per_second_t SelfControlledSwerveDriveSimulation::MaxAngularVelocity() const {
        return swerveDriveSimulation_.MaxAngularVelocity();
    }

    units::radians_per_second_squared_t SelfControlledSwerveDriveSimulation::MaxAngularAcceleration() const {
        return swerveDriveSimulation_.MaxAngularAcceleration(moduleSimulations_[0].GetDriveCurrentLimit());
    }
} // namespace maplesim::simulation::drivesims
