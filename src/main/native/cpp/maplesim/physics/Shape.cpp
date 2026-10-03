#include "pch.h"

#include "maplesim/physics/Shape.h"

#include <array>
#include <cassert>
#include <cstddef>
#include <numbers>
#include <utility>

#include <box2d/b2_settings.h>
#include <wpi/units/math.hpp>

namespace maplesim::physics {
    Shape::Shape(Geometry geometry)
        : geometry_(std::move(geometry)) {}

    Shape Shape::Circle(wpi::units::meter_t radius) {
        return Shape{CircleGeometry{radius, wpi::math::Translation2d{}}};
    }

    Shape Shape::Rectangle(wpi::units::meter_t width, wpi::units::meter_t height, const wpi::math::Transform2d& offset) {
        const std::array<wpi::math::Translation2d, 4> corners{
            wpi::math::Translation2d{-width / 2, -height / 2},
            wpi::math::Translation2d{width / 2, -height / 2},
            wpi::math::Translation2d{width / 2, height / 2},
            wpi::math::Translation2d{-width / 2, height / 2},
        };
        std::vector<wpi::math::Translation2d> vertices;
        vertices.reserve(corners.size());
        for (const wpi::math::Translation2d& corner : corners)
            vertices.push_back(offset.Translation() + corner.RotateBy(offset.Rotation()));
        return Shape{PolygonGeometry{std::move(vertices)}};
    }

    Shape Shape::Segment(const wpi::math::Translation2d& start, const wpi::math::Translation2d& end) {
        return Shape{SegmentGeometry{start, end}};
    }

    Shape Shape::Polygon(std::vector<wpi::math::Translation2d> vertices) {
        assert(vertices.size() >= 3 && vertices.size() <= b2_maxPolygonVertices);
        return Shape{PolygonGeometry{std::move(vertices)}};
    }

    wpi::units::square_meter_t Shape::GetArea() const {
        if (const CircleGeometry* circle = std::get_if<CircleGeometry>(&geometry_)) {
            return std::numbers::pi * circle->radius * circle->radius;
        }
        if (const PolygonGeometry* polygon = std::get_if<PolygonGeometry>(&geometry_)) {
            wpi::units::square_meter_t twiceSignedArea{0};
            const std::vector<wpi::math::Translation2d>& vertices = polygon->vertices;
            for (std::size_t i = 0; i < vertices.size(); i++) {
                const wpi::math::Translation2d& current = vertices[i];
                const wpi::math::Translation2d& next = vertices[(i + 1) % vertices.size()];
                twiceSignedArea += current.X() * next.Y() - next.X() * current.Y();
            }
            return wpi::units::math::abs(twiceSignedArea) / 2;
        }
        return wpi::units::square_meter_t{0};
    }
} // namespace maplesim::physics
