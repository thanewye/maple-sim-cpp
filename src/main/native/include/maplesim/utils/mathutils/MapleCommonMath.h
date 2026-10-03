#pragma once

#include <cstdint>

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>

namespace maplesim::utils::mathutils::MapleCommonMath {
    /** Test-only: reseeds the generator behind GenerateRandomNormal so scenarios are reproducible. */
    void SetSeed(std::uint64_t seed);

    [[nodiscard]] double GenerateRandomNormal(double mean, double stdDev);
    [[nodiscard]] double ConstrainMagnitude(double value, double maxMagnitude);
    [[nodiscard]] double LinearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x);
    [[nodiscard]] double LinearInterpretation(double x1, double y1, double x2, double y2, double x);

    /** Angle of the translation, or zero when it is too short to have one. */
    [[nodiscard]] wpi::math::Rotation2d GetAngle(const wpi::math::Translation2d& translation2d);
} // namespace maplesim::utils::mathutils::MapleCommonMath
