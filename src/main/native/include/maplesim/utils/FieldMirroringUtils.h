#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/units/length.hpp>

namespace maplesim::utils::FieldMirroringUtils {
    inline constexpr wpi::units::meter_t kFieldWidth{17.548};
    inline constexpr wpi::units::meter_t kFieldHeight{8.052};

    [[nodiscard]] wpi::math::Rotation2d ToCurrentAllianceRotation(const wpi::math::Rotation2d& rotationAtBlueSide);
    [[nodiscard]] wpi::math::Rotation2d Flip(const wpi::math::Rotation2d& rotation);
    [[nodiscard]] wpi::math::Translation2d ToCurrentAllianceTranslation(const wpi::math::Translation2d& translationAtBlueSide);
    [[nodiscard]] wpi::math::Translation2d Flip(const wpi::math::Translation2d& translation);
    [[nodiscard]] wpi::math::Pose3d Flip(const wpi::math::Pose3d& toFlip);
    [[nodiscard]] wpi::math::Translation3d ToCurrentAllianceTranslation(const wpi::math::Translation3d& translation3dAtBlueSide);
    [[nodiscard]] wpi::math::Pose2d ToCurrentAlliancePose(const wpi::math::Pose2d& poseAtBlueSide);
    [[nodiscard]] bool IsSidePresentedAsRed();
    [[nodiscard]] wpi::math::Rotation2d GetCurrentAllianceDriverStationFacing();
} // namespace maplesim::utils::FieldMirroringUtils
