#pragma once

#include <string>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>

namespace maplesim::simulation::gamepieces {
    /** A game piece tracked by the arena, either resting on the field or flying as a projectile. */
    class GamePiece {
    public:
        virtual ~GamePiece() = default;

        [[nodiscard]] virtual wpi::math::Pose3d GetPose3d() const = 0;
        [[nodiscard]] virtual const std::string& GetType() const = 0;
        [[nodiscard]] virtual wpi::math::Translation3d GetVelocity3dMPS() const = 0;
        virtual void TriggerHitTargetCallBack() = 0;
        /** True for pieces resting on the field, false for projectiles. */
        [[nodiscard]] virtual bool IsGrounded() const = 0;
    };
} // namespace maplesim::simulation::gamepieces
