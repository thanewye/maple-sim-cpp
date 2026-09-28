#pragma once

#include <units/area.h>
#include <units/mass.h>
#include <units/velocity.h>

#include "maplesim/physics/Shape.h"

class b2Fixture;

namespace maplesim::physics {
    class Body;

    using kilograms_per_square_meter_t = units::unit_t<units::compound_unit<units::kilograms, units::inverse<units::square_meters>>>;

    /** Defaults match dyn4j's BodyFixture, not Box2D's b2FixtureDef. */
    struct FixtureMaterial {
        double friction = 0.2;
        double restitution = 0.0;
        units::meters_per_second_t restitutionThreshold{1.0};
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
