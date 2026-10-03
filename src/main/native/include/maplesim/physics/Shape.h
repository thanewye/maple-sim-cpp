#pragma once

#include <variant>
#include <vector>

#include <wpi/math/geometry/Transform2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/units/area.hpp>
#include <wpi/units/length.hpp>

namespace maplesim::physics {
    struct CircleGeometry {
        wpi::units::meter_t radius;
        wpi::math::Translation2d center;
    };

    struct PolygonGeometry {
        std::vector<wpi::math::Translation2d> vertices;
    };

    struct SegmentGeometry {
        wpi::math::Translation2d start;
        wpi::math::Translation2d end;
    };

    class Shape {
    public:
        using Geometry = std::variant<CircleGeometry, PolygonGeometry, SegmentGeometry>;

        [[nodiscard]] static Shape Circle(wpi::units::meter_t radius);
        [[nodiscard]] static Shape Rectangle(wpi::units::meter_t width, wpi::units::meter_t height,
                                             const wpi::math::Transform2d& offset = wpi::math::Transform2d{});
        [[nodiscard]] static Shape Segment(const wpi::math::Translation2d& start, const wpi::math::Translation2d& end);
        [[nodiscard]] static Shape Polygon(std::vector<wpi::math::Translation2d> vertices);

        [[nodiscard]] wpi::units::square_meter_t GetArea() const;
        [[nodiscard]] const Geometry& GetGeometry() const { return geometry_; }

    private:
        explicit Shape(Geometry geometry);

        Geometry geometry_;
    };
} // namespace maplesim::physics
