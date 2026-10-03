#pragma once

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/torque.hpp>

namespace maplesim::simulation::motorsims {
    /** Final angular position and velocity of a simulated mechanism. */
    struct SimMotorState {
        wpi::units::radian_t mechanismAngularPosition{0.0};
        wpi::units::radians_per_second_t mechanismAngularVelocity{0.0};

        /** Applies electric torque, then friction that stops rather than reverses the mechanism, then integrates position. */
        void Step(wpi::units::newton_meter_t finalElectricTorque, wpi::units::newton_meter_t finalFrictionTorque, wpi::units::kilogram_square_meter_t loadMOI,
                  wpi::units::second_t dt);
    };
} // namespace maplesim::simulation::motorsims
