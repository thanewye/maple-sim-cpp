#include "pch.h"

#include "maplesim/physics/Shape.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <numbers>
#include <utility>

#include <box2d/b2_settings.h>
#include <units/math.h>

namespace maplesim::physics {
    Shape::Shape(Geometry geometry)
        : geometry_(std::move(geometry)) {}

    Shape Shape::Circle(units::meter_t radius) {
        return Shape{CircleGeometry{radius, frc::Translation2d{}}};
    }

    Shape Shape::Rectangle(units::meter_t width, units::meter_t height, const frc::Transform2d& offset) {
        const std::array<frc::Translation2d, 4> corners{
            frc::Translation2d{-width / 2, -height / 2},
            frc::Translation2d{width / 2, -height / 2},
            frc::Translation2d{width / 2, height / 2},
            frc::Translation2d{-width / 2, height / 2},
        };
        std::vector<frc::Translation2d> vertices;
        vertices.reserve(corners.size());
        for (const frc::Translation2d& corner : corners)
            vertices.push_back(offset.Translation() + corner.RotateBy(offset.Rotation()));
        return Shape{PolygonGeometry{std::move(vertices)}};
    }

    Shape Shape::Segment(const frc::Translation2d& start, const frc::Translation2d& end) {
        return Shape{SegmentGeometry{start, end}};
    }

    Shape Shape::Polygon(std::vector<frc::Translation2d> vertices) {
        assert(vertices.size() >= 3 && vertices.size() <= b2_maxPolygonVertices);
        return Shape{PolygonGeometry{std::move(vertices)}};
    }

    units::square_meter_t Shape::GetArea() const {
        if (const CircleGeometry* circle = std::get_if<CircleGeometry>(&geometry_)) {
            return std::numbers::pi * circle->radius * circle->radius;
        }
        if (const PolygonGeometry* polygon = std::get_if<PolygonGeometry>(&geometry_)) {
            units::square_meter_t twiceSignedArea{0};
            const std::vector<frc::Translation2d>& vertices = polygon->vertices;
            for (std::size_t i = 0; i < vertices.size(); i++) {
                const frc::Translation2d& current = vertices[i];
                const frc::Translation2d& next = vertices[(i + 1) % vertices.size()];
                twiceSignedArea += current.X() * next.Y() - next.X() * current.Y();
            }
            return units::math::abs(twiceSignedArea) / 2;
        }
        return units::square_meter_t{0};
    }
} // namespace maplesim::physics
