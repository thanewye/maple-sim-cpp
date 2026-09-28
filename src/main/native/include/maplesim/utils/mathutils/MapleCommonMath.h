#pragma once

#include <cstdint>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

namespace maplesim::utils::mathutils::MapleCommonMath {
    /** Test-only: reseeds the generator behind GenerateRandomNormal so scenarios are reproducible. */
    void SetSeed(std::uint64_t seed);

    [[nodiscard]] double GenerateRandomNormal(double mean, double stdDev);
    [[nodiscard]] double ConstrainMagnitude(double value, double maxMagnitude);
    [[nodiscard]] double LinearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x);
    [[nodiscard]] double LinearInterpretation(double x1, double y1, double x2, double y2, double x);

    /** Angle of the translation, or zero when it is too short to have one. */
    [[nodiscard]] frc::Rotation2d GetAngle(const frc::Translation2d& translation2d);
} // namespace maplesim::utils::mathutils::MapleCommonMath
