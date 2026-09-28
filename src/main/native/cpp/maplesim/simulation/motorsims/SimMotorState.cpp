#include "pch.h"

#include "maplesim/simulation/motorsims/SimMotorState.h"

#include <cmath>

namespace maplesim::simulation::motorsims {
    void SimMotorState::Step(units::newton_meter_t finalElectricTorque, units::newton_meter_t finalFrictionTorque, units::kilogram_square_meter_t loadMOI,
                             units::second_t dt) {
        double currentAngularPositionRadians = mechanismAngularPosition.value();
        double currentAngularVelocityRadiansPerSecond = mechanismAngularVelocity.value();
        const double electricTorqueNewtonsMeters = finalElectricTorque.value();
        const double frictionTorqueNewtonsMeters = finalFrictionTorque.value();
        const double loadMOIKgMetersSquared = loadMOI.value();
        const double dtSeconds = dt.value();

        currentAngularVelocityRadiansPerSecond += electricTorqueNewtonsMeters / loadMOIKgMetersSquared * dtSeconds;

        const double deltaAngularVelocityDueToFrictionRadPerSec =
            std::copysign(frictionTorqueNewtonsMeters, -currentAngularVelocityRadiansPerSecond) / loadMOIKgMetersSquared * dtSeconds;

        if ((currentAngularVelocityRadiansPerSecond + deltaAngularVelocityDueToFrictionRadPerSec) * currentAngularVelocityRadiansPerSecond <= 0)
            currentAngularVelocityRadiansPerSecond = 0;
        else currentAngularVelocityRadiansPerSecond += deltaAngularVelocityDueToFrictionRadPerSec;

        currentAngularPositionRadians += currentAngularVelocityRadiansPerSecond * dtSeconds;

        mechanismAngularPosition = units::radian_t{currentAngularPositionRadians};
        mechanismAngularVelocity = units::radians_per_second_t{currentAngularVelocityRadiansPerSecond};
    }
} // namespace maplesim::simulation::motorsims
