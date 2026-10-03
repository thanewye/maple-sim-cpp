#include "pch.h"

#include "maplesim/utils/FieldMirroringUtils.h"

#include <optional>

#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/units/angle.hpp>

namespace maplesim::utils::FieldMirroringUtils {
    wpi::math::Rotation2d ToCurrentAllianceRotation(const wpi::math::Rotation2d& rotationAtBlueSide) {
        return IsSidePresentedAsRed() ? Flip(rotationAtBlueSide) : rotationAtBlueSide;
    }

    wpi::math::Rotation2d Flip(const wpi::math::Rotation2d& rotation) {
        return rotation + wpi::math::Rotation2d{wpi::units::degree_t{180.0}};
    }

    wpi::math::Translation2d ToCurrentAllianceTranslation(const wpi::math::Translation2d& translationAtBlueSide) {
        return IsSidePresentedAsRed() ? Flip(translationAtBlueSide) : translationAtBlueSide;
    }

    wpi::math::Translation2d Flip(const wpi::math::Translation2d& translation) {
        return wpi::math::Translation2d{kFieldWidth - translation.X(), kFieldHeight - translation.Y()};
    }

    wpi::math::Pose3d Flip(const wpi::math::Pose3d& toFlip) {
        return wpi::math::Pose3d{wpi::math::Translation3d{kFieldWidth - toFlip.X(), kFieldHeight - toFlip.Y(), toFlip.Z()},
                                 toFlip.Rotation().RotateBy(wpi::math::Rotation3d{wpi::math::Rotation2d{wpi::units::degree_t{180}}})};
    }

    wpi::math::Translation3d ToCurrentAllianceTranslation(const wpi::math::Translation3d& translation3dAtBlueSide) {
        const wpi::math::Translation2d translation3dAtCurrentAlliance = ToCurrentAllianceTranslation(translation3dAtBlueSide.ToTranslation2d());
        if (IsSidePresentedAsRed())
            return wpi::math::Translation3d{translation3dAtCurrentAlliance.X(), translation3dAtCurrentAlliance.Y(), translation3dAtBlueSide.Z()};
        return translation3dAtBlueSide;
    }

    wpi::math::Pose2d ToCurrentAlliancePose(const wpi::math::Pose2d& poseAtBlueSide) {
        return wpi::math::Pose2d{ToCurrentAllianceTranslation(poseAtBlueSide.Translation()), ToCurrentAllianceRotation(poseAtBlueSide.Rotation())};
    }

    bool IsSidePresentedAsRed() {
        const std::optional<wpi::Alliance> alliance = wpi::MatchState::GetAlliance();
        return alliance.has_value() && alliance.value() == wpi::Alliance::RED;
    }

    wpi::math::Rotation2d GetCurrentAllianceDriverStationFacing() {
        return ToCurrentAllianceRotation(wpi::math::Rotation2d{});
    }
} // namespace maplesim::utils::FieldMirroringUtils
