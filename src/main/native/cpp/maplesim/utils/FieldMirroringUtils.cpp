#include "pch.h"

#include "maplesim/utils/FieldMirroringUtils.h"

#include <optional>

#include <frc/DriverStation.h>
#include <units/angle.h>

namespace maplesim::utils::FieldMirroringUtils {
    frc::Rotation2d ToCurrentAllianceRotation(const frc::Rotation2d& rotationAtBlueSide) {
        return IsSidePresentedAsRed() ? Flip(rotationAtBlueSide) : rotationAtBlueSide;
    }

    frc::Rotation2d Flip(const frc::Rotation2d& rotation) {
        return rotation + frc::Rotation2d{units::degree_t{180.0}};
    }

    frc::Translation2d ToCurrentAllianceTranslation(const frc::Translation2d& translationAtBlueSide) {
        return IsSidePresentedAsRed() ? Flip(translationAtBlueSide) : translationAtBlueSide;
    }

    frc::Translation2d Flip(const frc::Translation2d& translation) {
        return frc::Translation2d{kFieldWidth - translation.X(), kFieldHeight - translation.Y()};
    }

    frc::Pose3d Flip(const frc::Pose3d& toFlip) {
        return frc::Pose3d{frc::Translation3d{kFieldWidth - toFlip.X(), kFieldHeight - toFlip.Y(), toFlip.Z()},
                           toFlip.Rotation().RotateBy(frc::Rotation3d{frc::Rotation2d{units::degree_t{180}}})};
    }

    frc::Translation3d ToCurrentAllianceTranslation(const frc::Translation3d& translation3dAtBlueSide) {
        const frc::Translation2d translation3dAtCurrentAlliance = ToCurrentAllianceTranslation(translation3dAtBlueSide.ToTranslation2d());
        if (IsSidePresentedAsRed())
            return frc::Translation3d{translation3dAtCurrentAlliance.X(), translation3dAtCurrentAlliance.Y(), translation3dAtBlueSide.Z()};
        return translation3dAtBlueSide;
    }

    frc::Pose2d ToCurrentAlliancePose(const frc::Pose2d& poseAtBlueSide) {
        return frc::Pose2d{ToCurrentAllianceTranslation(poseAtBlueSide.Translation()), ToCurrentAllianceRotation(poseAtBlueSide.Rotation())};
    }

    bool IsSidePresentedAsRed() {
        const std::optional<frc::DriverStation::Alliance> alliance = frc::DriverStation::GetAlliance();
        return alliance.has_value() && alliance.value() == frc::DriverStation::Alliance::kRed;
    }

    frc::Rotation2d GetCurrentAllianceDriverStationFacing() {
        return ToCurrentAllianceRotation(frc::Rotation2d{});
    }
} // namespace maplesim::utils::FieldMirroringUtils
