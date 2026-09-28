#pragma once

#include <deque>
#include <vector>

#include <frc/geometry/Rotation2d.h>
#include <units/angular_velocity.h>

namespace maplesim::simulation::drivesims {
    /** Gyro that integrates the drivetrain's true angular velocity with measurement noise, idle drift and impact drift. */
    class GyroSimulation {
    public:
        GyroSimulation(double averageDriftingIn30SecsMotionlessDeg, double velocityMeasurementStandardDeviationPercent);

        void SetRotation(const frc::Rotation2d& currentRotation);
        [[nodiscard]] frc::Rotation2d GetGyroReading() const { return gyroReading_; }
        [[nodiscard]] units::radians_per_second_t GetMeasuredAngularVelocity() const;
        /** Readings from each sub-tick of the last robot period, oldest first. */
        [[nodiscard]] std::vector<frc::Rotation2d> GetCachedGyroReadings() const;

        void UpdateSimulationSubTick(units::radians_per_second_t actualAngularVelocity);

    private:
        [[nodiscard]] frc::Rotation2d GetDriftingDueToImpact(double actualAngularVelocityRadPerSec);
        [[nodiscard]] frc::Rotation2d GetGyroDTheta(double actualAngularVelocityRadPerSec);
        [[nodiscard]] frc::Rotation2d GetNoMotionDrifting() const;

        const double averageDriftingIn30SecsMotionlessDeg_;
        const double velocityMeasurementStandardDeviationPercent_;

        frc::Rotation2d gyroReading_;
        double measuredAngularVelocityRadPerSec_ = 0.0;
        double previousAngularVelocityRadPerSec_ = 0.0;
        std::deque<frc::Rotation2d> cachedRotations_;
    };
} // namespace maplesim::simulation::drivesims
