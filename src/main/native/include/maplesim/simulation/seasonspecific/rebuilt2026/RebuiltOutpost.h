#pragma once

#include <vector>

#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation3d.h>
#include <networktables/StructTopic.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "maplesim/simulation/Goal.h"

namespace maplesim::simulation::seasonspecific::rebuilt2026 {
    class Arena2026Rebuilt;

    /** A 2026 REBUILT outpost that stores fuel fed into it and can dump or throw it back onto the field. */
    class RebuiltOutpost : public Goal {
    public:
        RebuiltOutpost(Arena2026Rebuilt& arena, bool isBlue);

        void SimulationSubTick(int subTickNum) override;
        void Draw(std::vector<frc::Pose3d>& drawList) const override;

        void Reset();
        void ThrowForGoal();
        void Dump();
        void ThrowFuel(const frc::Rotation2d& yaw, units::radian_t pitch, units::meters_per_second_t speed);

    protected:
        [[nodiscard]] static const frc::Translation3d& RedOutpostPose();
        [[nodiscard]] static const frc::Translation3d& RedLaunchPose();
        [[nodiscard]] static const frc::Translation3d& RedDumpPose();
        [[nodiscard]] static const frc::Translation3d& BlueOutpostPose();
        [[nodiscard]] static const frc::Translation3d& BlueDumpPose();
        [[nodiscard]] static const frc::Translation3d& BlueLaunchPose();
        [[nodiscard]] static const frc::Translation3d& RedRenderPose();
        [[nodiscard]] static const frc::Translation3d& BlueRenderPose();

        void AddPoints() override;

        Arena2026Rebuilt& rebuiltArena_;

    private:
        nt::StructPublisher<frc::Pose3d> outpostPublisher_;
        nt::StructPublisher<frc::Pose3d> outpostThrowPublisher_;
        nt::StructPublisher<frc::Pose3d> outpostDumpPublisher_;
    };
} // namespace maplesim::simulation::seasonspecific::rebuilt2026
