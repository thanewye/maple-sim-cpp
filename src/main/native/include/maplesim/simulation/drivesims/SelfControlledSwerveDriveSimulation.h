#pragma once

#include <array>
#include <cstddef>
#include <vector>

#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/estimator/SwerveDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/units/acceleration.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>
#include <wpi/util/array.hpp>

#include "maplesim/simulation/drivesims/SwerveDriveSimulation.h"
#include "maplesim/simulation/motorsims/SimulatedMotorController.h"

namespace maplesim::simulation::drivesims {
    class SelfControlledSwerveDriveSimulation {
    public:
        static constexpr std::size_t kModuleCount = configs::kSwerveModuleCount;

        using ModulePositions = wpi::util::array<wpi::math::SwerveModulePosition, kModuleCount>;
        using CachedModulePositions = std::vector<ModulePositions>;
        using ModuleStates = wpi::util::array<wpi::math::SwerveModuleVelocity, kModuleCount>;

        class SelfControlledModuleSimulation {
        public:
            explicit SelfControlledModuleSimulation(SwerveModuleSimulation& moduleSimulation);

            SelfControlledModuleSimulation& WithSteerPID(const wpi::math::PIDController& steerController);
            SelfControlledModuleSimulation& WithCurrentLimits(wpi::units::ampere_t driveCurrentLimit, wpi::units::ampere_t steerCurrentLimit);

            [[nodiscard]] wpi::math::SwerveModuleVelocity OptimizeAndRunModuleState(wpi::math::SwerveModuleVelocity setPoint);
            void RunModuleState(const wpi::math::SwerveModuleVelocity& setPoint);
            void RunDriveMotorCharacterization(const wpi::math::Rotation2d& desiredModuleFacing, wpi::units::volt_t voltage);
            void RunSteerMotorCharacterization(wpi::units::volt_t voltage);

            [[nodiscard]] wpi::math::SwerveModuleVelocity GetMeasuredState() const;
            [[nodiscard]] wpi::math::SwerveModulePosition GetModulePosition() const;
            [[nodiscard]] SwerveModuleSimulation& GetModuleSimulation() const { return moduleSimulation_; }
            [[nodiscard]] wpi::units::ampere_t GetDriveCurrentLimit() const { return driveCurrentLimit_; }

        private:
            SwerveModuleSimulation& moduleSimulation_;
            wpi::units::ampere_t driveCurrentLimit_{60.0};
            wpi::math::PIDController steerController_{5.0, 0.0, 0.0};
            motorsims::SimulatedMotorController::GenericMotorController& driveMotor_;
            motorsims::SimulatedMotorController::GenericMotorController& steerMotor_;
        };

        explicit SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation);
        SelfControlledSwerveDriveSimulation(SwerveDriveSimulation& swerveDriveSimulation, const wpi::util::array<double, 3>& stateStdDevs,
                                            const wpi::util::array<double, 3>& visionMeasurementStdDevs);

        void Periodic();

        [[nodiscard]] ModulePositions GetLatestModulePositions() const;
        [[nodiscard]] CachedModulePositions GetCachedModulePositions() const;
        [[nodiscard]] wpi::math::Rotation2d GetRawGyroAngle() const;
        [[nodiscard]] wpi::math::Pose2d GetOdometryEstimatedPose() const;
        void ResetOdometry(const wpi::math::Pose2d& pose);
        void AddVisionEstimation(const wpi::math::Pose2d& robotPoseMeters, wpi::units::second_t timestamp);
        void AddVisionEstimation(const wpi::math::Pose2d& robotPoseMeters, wpi::units::second_t timestamp,
                                 const wpi::util::array<double, 3>& measurementStdDevs);

        void RunChassisSpeeds(wpi::math::ChassisVelocities chassisSpeeds, const wpi::math::Translation2d& centerOfRotationMeters, bool fieldCentricDrive,
                              bool discretizeSpeeds);
        void RunSwerveStates(const ModuleStates& setPoints);

        [[nodiscard]] ModuleStates GetMeasuredStates() const;
        [[nodiscard]] const ModuleStates& GetSetPointsOptimized() const { return setPointsOptimized_; }
        [[nodiscard]] wpi::math::ChassisVelocities GetMeasuredSpeedsFieldRelative(bool useGyroForAngularVelocity) const;
        [[nodiscard]] wpi::math::ChassisVelocities GetMeasuredSpeedsRobotRelative(bool useGyroForAngularVelocity) const;

        [[nodiscard]] SwerveDriveSimulation& GetDriveTrainSimulation() const { return swerveDriveSimulation_; }
        [[nodiscard]] wpi::math::Pose2d GetActualPoseInSimulationWorld() const;
        [[nodiscard]] wpi::math::ChassisVelocities GetActualSpeedsFieldRelative() const;
        [[nodiscard]] wpi::math::ChassisVelocities GetActualSpeedsRobotRelative() const;
        void SetSimulationWorldPose(const wpi::math::Pose2d& robotPose);

        SelfControlledSwerveDriveSimulation& WithSteerPID(const wpi::math::PIDController& steerController);
        SelfControlledSwerveDriveSimulation& WithCurrentLimits(wpi::units::ampere_t driveCurrentLimit, wpi::units::ampere_t steerCurrentLimit);

        [[nodiscard]] wpi::units::meters_per_second_t MaxLinearVelocity() const;
        [[nodiscard]] wpi::units::meters_per_second_squared_t MaxLinearAcceleration() const;
        [[nodiscard]] wpi::units::meter_t TrackWidthY() const;
        [[nodiscard]] wpi::units::meter_t TrackLengthX() const;
        [[nodiscard]] wpi::units::meter_t DriveBaseRadius() const;
        [[nodiscard]] wpi::units::radians_per_second_t MaxAngularVelocity() const;
        [[nodiscard]] wpi::units::radians_per_second_squared_t MaxAngularAcceleration() const;

    private:
        SwerveDriveSimulation& swerveDriveSimulation_;
        std::array<SelfControlledModuleSimulation, kModuleCount> moduleSimulations_;
        wpi::math::SwerveDriveKinematics<kModuleCount> kinematics_;
        wpi::math::SwerveDrivePoseEstimator<kModuleCount> poseEstimator_;
        ModuleStates setPointsOptimized_{wpi::util::empty_array};
    };
} // namespace maplesim::simulation::drivesims
