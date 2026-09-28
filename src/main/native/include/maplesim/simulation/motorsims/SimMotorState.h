#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>
#include <units/time.h>
#include <units/torque.h>

namespace maplesim::simulation::motorsims {
    /** Final angular position and velocity of a simulated mechanism. */
    struct SimMotorState {
        units::radian_t mechanismAngularPosition{0.0};
        units::radians_per_second_t mechanismAngularVelocity{0.0};

        /** Applies electric torque, then friction that stops rather than reverses the mechanism, then integrates position. */
        void Step(units::newton_meter_t finalElectricTorque, units::newton_meter_t finalFrictionTorque, units::kilogram_square_meter_t loadMOI,
                  units::second_t dt);
    };
} // namespace maplesim::simulation::motorsims
