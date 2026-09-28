#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/StructTopic.h>

#include "maplesim/simulation/Goal.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    class Arena2026Rebuilt;

    /** A 2026 REBUILT hub that scores fuel entering its opening and ejects it back onto the field. */
    class RebuiltHub : public Goal {
    public:
        static constexpr double kGoalRadius = 0.5969;

        [[nodiscard]] static const std::array<frc::Pose3d, 4>& RedShootPoses();
        /** Test-only: reseeds the generator that picks the ejection pose so scenarios are reproducible. */
        static void SetSeed(std::uint64_t seed);

        RebuiltHub(Arena2026Rebuilt& arena, bool isBlue);

        void Draw(std::vector<frc::Pose3d>& drawList) const override;

    protected:
        [[nodiscard]] static const frc::Translation3d& BlueHubPose();
        [[nodiscard]] static const frc::Translation3d& RedHubPose();
        [[nodiscard]] static const std::array<frc::Pose3d, 4>& BlueShootPoses();

        [[nodiscard]] bool CheckCollision(const gamepieces::GamePiece& gamePiece) const override;
        void AddPoints() override;

        Arena2026Rebuilt& rebuiltArena_;

    private:
        nt::StructPublisher<frc::Pose3d> hubPosePublisher_;
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
