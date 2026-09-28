#include "pch.h"

#include "maplesim/simulation/drivesims/GyroSimulation.h"

#include <cmath>
#include <numbers>

#include <units/angle.h>

#include "maplesim/simulation/SimulatedArena.h"
#include "maplesim/utils/mathutils/MapleCommonMath.h"

namespace maplesim::simulation::drivesims {
    namespace {
        constexpr double kAngularAccelerationThresholdStartDrifting = 500;
        constexpr double kDriftDueToImpactCoefficient = std::numbers::pi / 180.0;
    } // namespace

    GyroSimulation::GyroSimulation(double averageDriftingIn30SecsMotionlessDeg, double velocityMeasurementStandardDeviationPercent)
        : averageDriftingIn30SecsMotionlessDeg_(averageDriftingIn30SecsMotionlessDeg)
        , velocityMeasurementStandardDeviationPercent_(velocityMeasurementStandardDeviationPercent)
        , cachedRotations_(SimulatedArena::GetSimulationSubTicksIn1Period(), gyroReading_) {}

    void GyroSimulation::SetRotation(const frc::Rotation2d& currentRotation) {
        gyroReading_ = currentRotation;
    }

    units::radians_per_second_t GyroSimulation::GetMeasuredAngularVelocity() const {
        return units::radians_per_second_t{measuredAngularVelocityRadPerSec_};
    }

    std::vector<frc::Rotation2d> GyroSimulation::GetCachedGyroReadings() const {
        return {cachedRotations_.begin(), cachedRotations_.end()};
    }

    void GyroSimulation::UpdateSimulationSubTick(units::radians_per_second_t actualAngularVelocity) {
        const double actualAngularVelocityRadPerSec = actualAngularVelocity.value();
        const frc::Rotation2d driftingDueToImpact = GetDriftingDueToImpact(actualAngularVelocityRadPerSec);
        gyroReading_ = gyroReading_ + driftingDueToImpact;
        const frc::Rotation2d dTheta = GetGyroDTheta(actualAngularVelocityRadPerSec);
        gyroReading_ = gyroReading_ + dTheta;
        const frc::Rotation2d noMotionDrifting = GetNoMotionDrifting();
        gyroReading_ = gyroReading_ + noMotionDrifting;

        cachedRotations_.pop_front();
        cachedRotations_.push_back(gyroReading_);
    }

    frc::Rotation2d GyroSimulation::GetDriftingDueToImpact(double actualAngularVelocityRadPerSec) {
        const double angularAccelerationRadPerSecSq =
            (actualAngularVelocityRadPerSec - previousAngularVelocityRadPerSec_) / SimulatedArena::GetSimulationDt().value();
        const double driftingDueToImpactAbsVal =
            std::abs(angularAccelerationRadPerSecSq) > kAngularAccelerationThresholdStartDrifting
                ? std::abs(angularAccelerationRadPerSecSq) / kAngularAccelerationThresholdStartDrifting * kDriftDueToImpactCoefficient
                : 0;
        const double driftingDueToImpact = std::copysign(driftingDueToImpactAbsVal, -angularAccelerationRadPerSecSq);
        previousAngularVelocityRadPerSec_ = actualAngularVelocityRadPerSec;
        return frc::Rotation2d{units::radian_t{driftingDueToImpact}};
    }

    frc::Rotation2d GyroSimulation::GetGyroDTheta(double actualAngularVelocityRadPerSec) {
        measuredAngularVelocityRadPerSec_ = utils::mathutils::MapleCommonMath::GenerateRandomNormal(
            actualAngularVelocityRadPerSec, velocityMeasurementStandardDeviationPercent_ * std::abs(actualAngularVelocityRadPerSec));
        return frc::Rotation2d{units::radian_t{measuredAngularVelocityRadPerSec_ * SimulatedArena::GetSimulationDt().value()}};
    }

    frc::Rotation2d GyroSimulation::GetNoMotionDrifting() const {
        const double averageDrifting1Period = averageDriftingIn30SecsMotionlessDeg_ / 30 * SimulatedArena::GetSimulationDt().value();
        const double driftingInThisPeriod = utils::mathutils::MapleCommonMath::GenerateRandomNormal(0, averageDrifting1Period);
        return frc::Rotation2d{units::degree_t{driftingInThisPeriod}};
    }
} // namespace maplesim::simulation::drivesims
