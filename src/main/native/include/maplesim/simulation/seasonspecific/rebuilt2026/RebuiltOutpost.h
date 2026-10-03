#pragma once

#include <vector>

#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/nt/StructTopic.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/velocity.hpp>

#include "maplesim/simulation/Goal.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    class Arena2026Rebuilt;

    /** A 2026 REBUILT outpost that stores fuel fed into it and can dump or throw it back onto the field. */
    class RebuiltOutpost : public Goal {
    public:
        RebuiltOutpost(Arena2026Rebuilt& arena, bool isBlue);

        void SimulationSubTick(int subTickNum) override;
        void Draw(std::vector<wpi::math::Pose3d>& drawList) const override;

        void Reset();
        void ThrowForGoal();
        void Dump();
        void ThrowFuel(const wpi::math::Rotation2d& yaw, wpi::units::radian_t pitch, wpi::units::meters_per_second_t speed);

    protected:
        [[nodiscard]] static const wpi::math::Translation3d& RedOutpostPose();
        [[nodiscard]] static const wpi::math::Translation3d& RedLaunchPose();
        [[nodiscard]] static const wpi::math::Translation3d& RedDumpPose();
        [[nodiscard]] static const wpi::math::Translation3d& BlueOutpostPose();
        [[nodiscard]] static const wpi::math::Translation3d& BlueDumpPose();
        [[nodiscard]] static const wpi::math::Translation3d& BlueLaunchPose();
        [[nodiscard]] static const wpi::math::Translation3d& RedRenderPose();
        [[nodiscard]] static const wpi::math::Translation3d& BlueRenderPose();

        void AddPoints() override;

        Arena2026Rebuilt& rebuiltArena_;

    private:
        wpi::nt::StructPublisher<wpi::math::Pose3d> outpostPublisher_;
        wpi::nt::StructPublisher<wpi::math::Pose3d> outpostThrowPublisher_;
        wpi::nt::StructPublisher<wpi::math::Pose3d> outpostDumpPublisher_;
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
