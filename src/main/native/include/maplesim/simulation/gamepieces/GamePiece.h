#pragma once

#include <string>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>

namespace maplesim::simulation::gamepieces {
    /** A game piece tracked by the arena, either resting on the field or flying as a projectile. */
    class GamePiece {
    public:
        virtual ~GamePiece() = default;

        [[nodiscard]] virtual frc::Pose3d GetPose3d() const = 0;
        [[nodiscard]] virtual const std::string& GetType() const = 0;
        [[nodiscard]] virtual frc::Translation3d GetVelocity3dMPS() const = 0;
        virtual void TriggerHitTargetCallBack() = 0;
        /** True for pieces resting on the field, false for projectiles. */
        [[nodiscard]] virtual bool IsGrounded() const = 0;
    };
} // namespace maplesim::simulation::gamepieces
