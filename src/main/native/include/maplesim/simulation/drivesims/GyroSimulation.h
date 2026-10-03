#pragma once

#include <deque>
#include <vector>

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/angular_velocity.hpp>

namespace maplesim::simulation::drivesims {
    /** Gyro that integrates the drivetrain's true angular velocity with measurement noise, idle drift and impact drift. */
    class GyroSimulation {
    public:
        GyroSimulation(double averageDriftingIn30SecsMotionlessDeg, double velocityMeasurementStandardDeviationPercent);

        void SetRotation(const wpi::math::Rotation2d& currentRotation);
        [[nodiscard]] wpi::math::Rotation2d GetGyroReading() const { return gyroReading_; }
        [[nodiscard]] wpi::units::radians_per_second_t GetMeasuredAngularVelocity() const;
        /** Readings from each sub-tick of the last robot period, oldest first. */
        [[nodiscard]] std::vector<wpi::math::Rotation2d> GetCachedGyroReadings() const;

        void UpdateSimulationSubTick(wpi::units::radians_per_second_t actualAngularVelocity);

    private:
        [[nodiscard]] wpi::math::Rotation2d GetDriftingDueToImpact(double actualAngularVelocityRadPerSec);
        [[nodiscard]] wpi::math::Rotation2d GetGyroDTheta(double actualAngularVelocityRadPerSec);
        [[nodiscard]] wpi::math::Rotation2d GetNoMotionDrifting() const;

        const double averageDriftingIn30SecsMotionlessDeg_;
        const double velocityMeasurementStandardDeviationPercent_;

        wpi::math::Rotation2d gyroReading_;
        double measuredAngularVelocityRadPerSec_ = 0.0;
        double previousAngularVelocityRadPerSec_ = 0.0;
        std::deque<wpi::math::Rotation2d> cachedRotations_;
    };
} // namespace maplesim::simulation::drivesims
