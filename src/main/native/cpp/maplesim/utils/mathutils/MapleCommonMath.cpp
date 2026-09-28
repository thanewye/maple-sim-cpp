#include "pch.h"

#include "maplesim/utils/mathutils/MapleCommonMath.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

namespace maplesim::utils::mathutils::MapleCommonMath {
    namespace {
        [[nodiscard]] std::mt19937_64& GetRandom() {
            static std::mt19937_64 random{std::random_device{}()};
            return random;
        }

        [[nodiscard]] double NextDouble() {
            return std::uniform_real_distribution<double>{0.0, 1.0}(GetRandom());
        }
    } // namespace

    void SetSeed(std::uint64_t seed) {
        GetRandom().seed(seed);
    }

    double GenerateRandomNormal(double mean, double stdDev) {
        const double u1 = NextDouble();
        const double u2 = NextDouble();
        const double z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
        return z0 * stdDev + mean;
    }

    double ConstrainMagnitude(double value, double maxMagnitude) {
        return std::copysign(std::min(std::abs(value), std::abs(maxMagnitude)), value);
    }

    double LinearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x) {
        const double minX = std::min(x1, x2);
        const double maxX = std::max(x1, x2);
        return LinearInterpretation(x1, y1, x2, y2, std::min(maxX, std::max(minX, x)));
    }

    double LinearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    frc::Rotation2d GetAngle(const frc::Translation2d& translation2d) {
        const double tooSmall = 1e-6;
        return translation2d.Norm().value() < tooSmall ? frc::Rotation2d{} : translation2d.Angle();
    }
} // namespace maplesim::utils::mathutils::MapleCommonMath
