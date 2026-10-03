#include <cmath>
#include <memory>

#include <wpi/hal/DriverStationTypes.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/simulation/DriverStationSim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/simulation/SimHooks.hpp>

#include "gtest/gtest.h"
#include "maplesim/simulation/Goal.h"
#include "maplesim/simulation/drivesims/SelfControlledSwerveDriveSimulation.h"
#include "maplesim/simulation/motorsims/SimulatedBattery.h"
#include "maplesim/simulation/seasonspecific/evergreen/ArenaEvergreen.h"
#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"
#include "maplesim/utils/FieldMirroringUtils.h"
#include "maplesim/utils/mathutils/MapleCommonMath.h"
#include "maplesim/utils/mathutils/SwerveStateProjection.h"

namespace {
    namespace simulation = maplesim::simulation;
    namespace drivesims = simulation::drivesims;
    namespace units = wpi::units;

    class MigrationTest : public ::testing::Test {
    protected:
        void SetUp() override {
            wpi::sim::PauseTiming();
            wpi::sim::DriverStationSim::ResetData();
            wpi::sim::DriverStationSim::SetDsAttached(true);
            wpi::sim::DriverStationSim::NotifyNewData();
            wpi::sim::RoboRioSim::ResetData();
        }

        void TearDown() override {
            wpi::sim::ResumeTiming();
            wpi::sim::DriverStationSim::ResetData();
            wpi::sim::DriverStationSim::NotifyNewData();
        }
    };

    class OrientedGamePiece final : public simulation::gamepieces::GamePiece {
    public:
        wpi::math::Pose3d pose;
        std::string type = "Fuel";

        wpi::math::Pose3d GetPose3d() const override { return pose; }
        const std::string& GetType() const override { return type; }
        wpi::math::Translation3d GetVelocity3dMPS() const override { return {}; }
        void TriggerHitTargetCallBack() override {}
        bool IsGrounded() const override { return false; }
    };

    TEST_F(MigrationTest, GoalRotationTolerancePreservesRelativeOrientation) {
        const wpi::math::Rotation3d expected{units::degree_t{20}, units::degree_t{30}, units::degree_t{40}};
        const auto accepts = simulation::Goal::AbsoluteAngle(expected, units::degree_t{10});
        OrientedGamePiece piece;
        piece.pose = wpi::math::Pose3d{{}, expected};
        EXPECT_TRUE(accepts(piece));
        piece.pose = wpi::math::Pose3d{{}, expected.RotateBy(wpi::math::Rotation3d{units::degree_t{0}, units::degree_t{0}, units::degree_t{30}})};
        EXPECT_FALSE(accepts(piece));
    }

    TEST_F(MigrationTest, MotorVoltageAndVelocityConversionsRoundTrip) {
        const auto config = drivesims::configs::DriveTrainSimulationConfig::Default();
        auto module = config.swerveModuleSimulationFactories[0]();
        const auto& motor = module->config.driveMotorConfigs;
        const units::ampere_t current{10};
        const units::radians_per_second_t velocity{20};
        const auto voltage = motor.CalculateVoltage(current, velocity);
        EXPECT_NEAR(motor.CalculateMechanismVelocity(current, voltage).value(), velocity.value(), 1e-9);
        EXPECT_NEAR(motor.CalculateCurrent(velocity, voltage).value(), current.value(), 1e-9);
    }

    TEST_F(MigrationTest, TranslationAngleHandlesZeroAndNonzeroVectors) {
        using maplesim::utils::mathutils::MapleCommonMath::GetAngle;
        EXPECT_DOUBLE_EQ(GetAngle(wpi::math::Translation2d{}).Radians().value(), 0.0);
        EXPECT_DOUBLE_EQ(GetAngle(wpi::math::Translation2d{units::meter_t{1e-8}, units::meter_t{1e-8}}).Radians().value(), 0.0);
        EXPECT_NEAR(GetAngle(wpi::math::Translation2d{units::meter_t{0}, units::meter_t{1}}).Degrees().value(), 90.0, 1e-9);
    }

    TEST_F(MigrationTest, ModuleOptimizationReturnsTheReversedSetpoint) {
        const auto config = drivesims::configs::DriveTrainSimulationConfig::Default();
        auto module = config.swerveModuleSimulationFactories[0]();
        drivesims::SelfControlledSwerveDriveSimulation::SelfControlledModuleSimulation controller{*module};
        const auto facing = module->GetSteerAbsoluteFacing();
        const wpi::math::SwerveModuleVelocity requested{units::meters_per_second_t{2.0}, facing + wpi::math::Rotation2d{units::degree_t{180}}};
        const auto optimized = controller.OptimizeAndRunModuleState(requested);
        EXPECT_DOUBLE_EQ(optimized.velocity.value(), -2.0);
        EXPECT_NEAR((optimized.angle - facing).Radians().value(), 0.0, 1e-9);
        EXPECT_NEAR(maplesim::utils::mathutils::SwerveStateProjection::Project(optimized, facing).value(), -2.0, 1e-9);
    }

    TEST_F(MigrationTest, AllianceMirroringUsesMatchState) {
        const wpi::math::Translation2d translation{units::meter_t{2}, units::meter_t{3}};
        wpi::sim::DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::BLUE_1);
        wpi::sim::DriverStationSim::NotifyNewData();
        EXPECT_FALSE(maplesim::utils::FieldMirroringUtils::IsSidePresentedAsRed());
        EXPECT_EQ(maplesim::utils::FieldMirroringUtils::ToCurrentAllianceTranslation(translation), translation);
        wpi::sim::DriverStationSim::SetAllianceStationId(wpi::hal::AllianceStationID::RED_1);
        wpi::sim::DriverStationSim::NotifyNewData();
        EXPECT_TRUE(maplesim::utils::FieldMirroringUtils::IsSidePresentedAsRed());
        const auto mirrored = maplesim::utils::FieldMirroringUtils::ToCurrentAllianceTranslation(translation);
        EXPECT_NEAR(mirrored.X().value(), 15.548, 1e-9);
        EXPECT_NEAR(mirrored.Y().value(), 5.052, 1e-9);
    }

    TEST_F(MigrationTest, ScoringUsesAutonomousAndTeleopState) {
        simulation::seasonspecific::evergreen::ArenaEvergreen arena{false};
        wpi::sim::DriverStationSim::SetRobotMode(wpi::hal::RobotMode::AUTONOMOUS);
        wpi::sim::DriverStationSim::NotifyNewData();
        arena.AddToScore(true, 3);
        EXPECT_DOUBLE_EQ(arena.blueScoringBreakdown.at("Auto/AutoScore"), 3.0);
        wpi::sim::DriverStationSim::SetRobotMode(wpi::hal::RobotMode::TELEOPERATED);
        wpi::sim::DriverStationSim::NotifyNewData();
        arena.AddToScore(true, 2);
        EXPECT_EQ(arena.GetScore(wpi::Alliance::BLUE), 5);
        EXPECT_DOUBLE_EQ(arena.blueScoringBreakdown.at("TeleopScore"), 2.0);
    }

    TEST_F(MigrationTest, BatteryVoltageReachesHalAndNetworkTables) {
        using simulation::motorsims::SimulatedBattery;
        auto appliance = SimulatedBattery::AddElectricalAppliances([] { return units::ampere_t{50}; });
        for (int tick = 0; tick < 60; ++tick)
            SimulatedBattery::SimulationSubTick();
        EXPECT_NEAR(SimulatedBattery::GetBatteryVoltage().value(), 12.5, 1e-9);
        EXPECT_NEAR(wpi::sim::RoboRioSim::GetVInVoltage().value(), 12.5, 1e-9);
        EXPECT_NEAR(wpi::nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetNumber("BatterySim/BatteryVoltage (Volts)", -1), 12.5, 1e-9);
        appliance.Disconnect();
        for (int tick = 0; tick < 60; ++tick)
            SimulatedBattery::SimulationSubTick();
        EXPECT_NEAR(SimulatedBattery::GetBatteryVoltage().value(), 13.5, 1e-9);
    }

    TEST_F(MigrationTest, SwervePhysicsAndPoseEstimationAdvanceTogether) {
        simulation::seasonspecific::evergreen::ArenaEvergreen arena{false};
        auto& drive = arena.AddDriveTrainSimulation(
            std::make_unique<drivesims::SwerveDriveSimulation>(drivesims::configs::DriveTrainSimulationConfig::Default(), wpi::math::Pose2d{}));
        drivesims::SelfControlledSwerveDriveSimulation controller{drive};
        for (int period = 0; period < 150; ++period) {
            controller.RunChassisSpeeds(
                wpi::math::ChassisVelocities{units::meters_per_second_t{1}, units::meters_per_second_t{0}, units::radians_per_second_t{0}}, {}, false, true);
            arena.SimulationPeriodic();
            wpi::sim::StepTiming(units::second_t{0.02});
            controller.Periodic();
        }
        const auto actual = controller.GetActualPoseInSimulationWorld();
        const auto estimated = controller.GetOdometryEstimatedPose();
        EXPECT_GT(actual.X().value(), 1.0);
        EXPECT_NEAR(actual.Y().value(), 0.0, 0.1);
        EXPECT_NEAR(estimated.X().value(), actual.X().value(), 0.2);
        EXPECT_TRUE(std::isfinite(controller.GetMeasuredSpeedsRobotRelative(true).vx.value()));
    }

    TEST_F(MigrationTest, RebuiltArenaStillPlacesAndStepsFuel) {
        simulation::seasonspecific::rebuilt2026::Arena2026Rebuilt arena;
        arena.ResetFieldForAuto();
        EXPECT_FALSE(arena.GamePiecesOnField().empty());
        for (int period = 0; period < 5; ++period)
            arena.SimulationPeriodic();
        EXPECT_FALSE(arena.GetGamePiecesPosesByType("Fuel").empty());
    }
}
