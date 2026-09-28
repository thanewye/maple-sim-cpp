#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/array.h>

#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"
#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

namespace maplesim::simulation::drivesims {
    class SelfControlledSwerveDriveSimulation {
    public:
        static constexpr std::size_t kModuleCount = configs::kSwerveModuleCount;

        using ModulePositions = wpi::array<frc::SwerveModulePosition, kModuleCount>;
        using CachedModulePositions = std::vector<ModulePositions>;
        using ModuleStates = wpi::array<frc::SwerveModuleState, kModuleCount>;

        class SelfControlledModuleSimulation {
        public:
            explicit SelfControlledModuleSimulation(SwerveModuleSimulation& moduleSimulation);

            SelfControlledModuleSimulation& WithSteerPID(const frc::PIDController& steerController);
            SelfControlledModuleSimulation& WithCurrentLimits(units::ampere_t driveCurrentLimit, units::ampere_t steerCurrentLimit);

            [[nodiscard]] frc::SwerveModuleState OptimizeAndRunModuleState(frc::SwerveModuleState setPoint);
            void RunModuleState(const frc::SwerveModuleState& setPoint);
            void RunDriveMotorCharacterization(const frc::Rotation2d& desiredModuleFacing, units::volt_t voltage);
            void RunSteerMotorCharacterization(units::volt_t voltage);

            [[nodiscard]] frc::SwerveModuleState GetMeasuredState() const;
            [[nodiscard]] frc::SwerveModulePosition GetModulePosition() const;
            [[nodiscard]] SwerveModuleSimulation& GetModuleSimulation() const { return moduleSimulation_; }
            [[nodiscard]] units::ampere_t GetDriveCurrentLimit() const { return driveCurrentLimit_; }

        private:
            SwerveModuleSimulation& moduleSimulation_;
            units::ampere_t driveCurrentLimit_{60.0};
            frc::PIDController steerController_{5.0, 0.0, 0.0};
            motorsims::SimulatedMotorController::GenericMotorController& driveMotor_;
            motorsims::SimulatedMotorController::GenericMotorController& steerMotor_;
        };

        explicit SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation);
        SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation, const wpi::array<double, 3>& stateStdDevs,
                                            const wpi::array<double, 3>& visionMeasurementStdDevs);

        void Periodic();

        [[nodiscard]] ModulePositions GetLatestModulePositions() const;
        [[nodiscard]] CachedModulePositions GetCachedModulePositions() const;
        [[nodiscard]] frc::Rotation2d GetRawGyroAngle() const;
        [[nodiscard]] frc::Pose2d GetOdometryEstimatedPose() const;
        void ResetOdometry(const frc::Pose2d& pose);
        void AddVisionEstimation(const frc::Pose2d& robotPoseMeters, units::second_t timestamp);
        void AddVisionEstimation(const frc::Pose2d& robotPoseMeters, units::second_t timestamp, const wpi::array<double, 3>& measurementStdDevs);

        void RunChassisSpeeds(frc::ChassisSpeeds chassisSpeeds, const frc::Translation2d& centerOfRotationMeters, bool fieldCentricDrive,
                              bool discretizeSpeeds);
        void RunSwerveStates(const ModuleStates& setPoints);

        [[nodiscard]] ModuleStates GetMeasuredStates() const;
        [[nodiscard]] const ModuleStates& GetSetPointsOptimized() const { return setPointsOptimized_; }
        [[nodiscard]] frc::ChassisSpeeds GetMeasuredSpeedsFieldRelative(bool useGyroForAngularVelocity) const;
        [[nodiscard]] frc::ChassisSpeeds GetMeasuredSpeedsRobotRelative(bool useGyroForAngularVelocity) const;

        [[nodiscard]] SwerveDriveSimulation& GetDriveTrainSimulation() const { return swerveDriveSimulation_; }
        [[nodiscard]] frc::Pose2d GetActualPoseInSimulationWorld() const;
        [[nodiscard]] frc::ChassisSpeeds GetActualSpeedsFieldRelative() const;
        [[nodiscard]] frc::ChassisSpeeds GetActualSpeedsRobotRelative() const;
        void SetSimulationWorldPose(const frc::Pose2d& robotPose);

        SelfControlledSwerveDriveSimulation& WithSteerPID(const frc::PIDController& steerController);
        SelfControlledSwerveDriveSimulation& WithCurrentLimits(units::ampere_t driveCurrentLimit, units::ampere_t steerCurrentLimit);

        [[nodiscard]] units::meters_per_second_t MaxLinearVelocity() const;
        [[nodiscard]] units::meters_per_second_squared_t MaxLinearAcceleration() const;
        [[nodiscard]] units::meter_t TrackWidthY() const;
        [[nodiscard]] units::meter_t TrackLengthX() const;
        [[nodiscard]] units::meter_t DriveBaseRadius() const;
        [[nodiscard]] units::radians_per_second_t MaxAngularVelocity() const;
        [[nodiscard]] units::radians_per_second_squared_t MaxAngularAcceleration() const;

    private:
        SwerveDriveSimulation& swerveDriveSimulation_;
        std::array<SelfControlledModuleSimulation, kModuleCount> moduleSimulations_;
        frc::SwerveDriveKinematics<kModuleCount> kinematics_;
        frc::SwerveDrivePoseEstimator<kModuleCount> poseEstimator_;
        ModuleStates setPointsOptimized_{wpi::empty_array};
    };
} // namespace maplesim::simulation::drivesims
