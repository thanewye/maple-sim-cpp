#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <units/length.h>

namespace maplesim::utils::FieldMirroringUtils {
    inline constexpr units::meter_t kFieldWidth{17.548};
    inline constexpr units::meter_t kFieldHeight{8.052};

    [[nodiscard]] frc::Rotation2d ToCurrentAllianceRotation(const frc::Rotation2d& rotationAtBlueSide);
    [[nodiscard]] frc::Rotation2d Flip(const frc::Rotation2d& rotation);
    [[nodiscard]] frc::Translation2d ToCurrentAllianceTranslation(const frc::Translation2d& translationAtBlueSide);
    [[nodiscard]] frc::Translation2d Flip(const frc::Translation2d& translation);
    [[nodiscard]] frc::Pose3d Flip(const frc::Pose3d& toFlip);
    [[nodiscard]] frc::Translation3d ToCurrentAllianceTranslation(const frc::Translation3d& translation3dAtBlueSide);
    [[nodiscard]] frc::Pose2d ToCurrentAlliancePose(const frc::Pose2d& poseAtBlueSide);
    [[nodiscard]] bool IsSidePresentedAsRed();
    [[nodiscard]] frc::Rotation2d GetCurrentAllianceDriverStationFacing();
} // namespace maplesim::utils::FieldMirroringUtils
