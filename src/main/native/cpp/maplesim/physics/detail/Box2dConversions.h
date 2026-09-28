#pragma once

#include <array>
#include <cstddef>
#include <type_traits>
#include <variant>

#include <box2d/b2_circle_shape.h>
#include <box2d/b2_edge_shape.h>
#include <box2d/b2_math.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_settings.h>
#include <frc/geometry/Translation2d.h>

#include "maplesim/physics/Shape.h"

namespace maplesim::physics::detail {
    [[nodiscard]] inline b2Vec2 ToB2Vec2(const frc::Translation2d& translation) {
        return b2Vec2{static_cast<float>(translation.X().value()), static_cast<float>(translation.Y().value())};
    }

    [[nodiscard]] inline frc::Translation2d ToTranslation2d(const b2Vec2& vector) {
        return frc::Translation2d{units::meter_t{vector.x}, units::meter_t{vector.y}};
    }

    /** Box2D's hull welds points within 4 * linearSlop, collapsing thin rectangles that dyn4j accepts, so assign them directly. */
    inline void SetConvexPolygonWithoutHullWelding(b2PolygonShape& polygon, const b2Vec2* points, int32 count) {
        float twiceSignedArea = 0.0f;
        for (int32 i = 0; i < count; i++)
            twiceSignedArea += b2Cross(points[i], points[(i + 1) % count]);
        const bool isCounterClockwise = twiceSignedArea > 0.0f;

        b2Vec2 centroid{0.0f, 0.0f};
        polygon.m_count = count;
        for (int32 i = 0; i < count; i++) {
            polygon.m_vertices[i] = points[isCounterClockwise ? i : count - 1 - i];
            centroid += polygon.m_vertices[i];
        }
        for (int32 i = 0; i < count; i++) {
            const b2Vec2 edge = polygon.m_vertices[(i + 1) % count] - polygon.m_vertices[i];
            polygon.m_normals[i] = b2Cross(edge, 1.0f);
            polygon.m_normals[i].Normalize();
        }
        polygon.m_centroid = (1.0f / static_cast<float>(count)) * centroid;
    }

    template<class Visitor> void VisitB2Shape(const Shape& shape, Visitor&& visitor) {
        std::visit(
            [&visitor](const auto& geometry) {
                using GeometryType = std::decay_t<decltype(geometry)>;
                if constexpr (std::is_same_v<GeometryType, CircleGeometry>) {
                    b2CircleShape circle;
                    circle.m_radius = static_cast<float>(geometry.radius.value());
                    circle.m_p = ToB2Vec2(geometry.center);
                    visitor(static_cast<const b2Shape&>(circle));
                } else if constexpr (std::is_same_v<GeometryType, PolygonGeometry>) {
                    std::array<b2Vec2, b2_maxPolygonVertices> points{};
                    for (std::size_t i = 0; i < geometry.vertices.size(); i++)
                        points[i] = ToB2Vec2(geometry.vertices[i]);
                    b2PolygonShape polygon;
                    const auto count = static_cast<int32>(geometry.vertices.size());
                    if (!polygon.Set(points.data(), count)) SetConvexPolygonWithoutHullWelding(polygon, points.data(), count);
                    visitor(static_cast<const b2Shape&>(polygon));
                } else {
                    b2EdgeShape edge;
                    edge.SetTwoSided(ToB2Vec2(geometry.start), ToB2Vec2(geometry.end));
                    visitor(static_cast<const b2Shape&>(edge));
                }
            },
            shape.GetGeometry());
    }
} // namespace maplesim::physics::detail
