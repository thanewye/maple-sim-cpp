#pragma once

#include <variant>
#include <vector>

#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/area.h>
#include <units/length.h>

namespace maplesim::physics {
    struct CircleGeometry {
        units::meter_t radius;
        frc::Translation2d center;
    };

    struct PolygonGeometry {
        std::vector<frc::Translation2d> vertices;
    };

    struct SegmentGeometry {
        frc::Translation2d start;
        frc::Translation2d end;
    };

    class Shape {
    public:
        using Geometry = std::variant<CircleGeometry, PolygonGeometry, SegmentGeometry>;

        [[nodiscard]] static Shape Circle(units::meter_t radius);
        [[nodiscard]] static Shape Rectangle(units::meter_t width, units::meter_t height, const frc::Transform2d& offset = frc::Transform2d{});
        [[nodiscard]] static Shape Segment(const frc::Translation2d& start, const frc::Translation2d& end);
        [[nodiscard]] static Shape Polygon(std::vector<frc::Translation2d> vertices);

        [[nodiscard]] units::square_meter_t GetArea() const;
        [[nodiscard]] const Geometry& GetGeometry() const { return geometry_; }

    private:
        explicit Shape(Geometry geometry);

        Geometry geometry_;
    };
} // namespace maplesim::physics
