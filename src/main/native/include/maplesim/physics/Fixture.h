#pragma once

#include <wpi/units/area.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/physics/Shape.h"

class b2Fixture;

namespace maplesim::physics {
    class Body;

    using kilograms_per_square_meter_t = wpi::units::unit_t<wpi::units::compound_unit<wpi::units::kilograms, wpi::units::inverse<wpi::units::square_meters>>>;

    /** Defaults match dyn4j's BodyFixture, not Box2D's b2FixtureDef. */
    struct FixtureMaterial {
        double friction = 0.2;
        double restitution = 0.0;
        wpi::units::meters_per_second_t restitutionThreshold{1.0};
        kilograms_per_square_meter_t density{1.0};
    };

    class Fixture {
    public:
        Fixture(Shape shape, FixtureMaterial material);
        virtual ~Fixture();

        Fixture(const Fixture&) = delete;
        Fixture& operator=(const Fixture&) = delete;

        [[nodiscard]] const Shape& GetShape() const { return shape_; }
        [[nodiscard]] const FixtureMaterial& GetMaterial() const { return material_; }
        [[nodiscard]] Body* GetBody() const { return body_; }

    private:
        friend class Body;

        Shape shape_;
        FixtureMaterial material_;
        Body* body_ = nullptr;
        b2Fixture* b2Fixture_ = nullptr;
    };
} // namespace maplesim::physics
