#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/units/force.hpp>
#include <wpi/units/math.hpp>
#include <wpi/units/velocity.hpp>

namespace maplesim::physics {
    template<class Unit> struct Vector2d {
        Unit x{0};
        Unit y{0};

        [[nodiscard]] static Vector2d FromPolar(Unit magnitude, const wpi::math::Rotation2d& direction) {
            return Vector2d{magnitude * direction.Cos(), magnitude * direction.Sin()};
        }

        [[nodiscard]] Unit Norm() const { return wpi::units::math::hypot(x, y); }

        /** Direction of this vector; zero for the zero vector, matching dyn4j's atan2 rather than WPILib's error path. */
        [[nodiscard]] wpi::math::Rotation2d Angle() const { return wpi::math::Rotation2d{wpi::units::math::atan2(y, x)}; }

        [[nodiscard]] Vector2d operator+(const Vector2d& other) const { return Vector2d{x + other.x, y + other.y}; }
        [[nodiscard]] Vector2d operator-(const Vector2d& other) const { return Vector2d{x - other.x, y - other.y}; }
        [[nodiscard]] Vector2d operator-() const { return Vector2d{-x, -y}; }
        [[nodiscard]] Vector2d operator*(double scalar) const { return Vector2d{x * scalar, y * scalar}; }
        [[nodiscard]] friend Vector2d operator*(double scalar, const Vector2d& vector) { return vector * scalar; }

        Vector2d& operator+=(const Vector2d& other) {
            x += other.x;
            y += other.y;
            return *this;
        }

        [[nodiscard]] bool operator==(const Vector2d& other) const = default;
    };

    using Force2d = Vector2d<wpi::units::newton_t>;
    using LinearVelocity2d = Vector2d<wpi::units::meters_per_second_t>;
} // namespace maplesim::physics
