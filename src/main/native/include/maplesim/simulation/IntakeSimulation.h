#pragma once

#include <deque>
#include <functional>
#include <memory>
#include <string>

#include <units/length.h>

#include "maplesim/physics/Fixture.h"
#include "maplesim/physics/Shape.h"
#include "maplesim/physics/World.h"
#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"

namespace maplesim::simulation {
    class SimulatedArena;

    /** A fixture on the drivetrain that collects game pieces of one type on contact while running. */
    class IntakeSimulation : public physics::Fixture {
    public:
        enum class IntakeSide { kFront, kLeft, kRight, kBack };

        /** Flags touching game pieces of the targeted type for removal after the physics step. */
        class GamePieceContactListener {
        public:
            explicit GamePieceContactListener(IntakeSimulation& intakeSimulation)
                : intakeSimulation_(intakeSimulation) {}

            void operator()(const physics::Contact& contact) const;

        private:
            void FlagGamePieceForRemoval(gamepieces::GamePieceOnFieldSimulation& gamePiece) const;

            IntakeSimulation& intakeSimulation_;
        };

        static IntakeSimulation& InTheFrameIntake(const std::string& targetedGamePieceType, drivesims::AbstractDriveTrainSimulation& driveTrainSimulation,
                                                  units::meter_t width, IntakeSide side, int capacity);
        static IntakeSimulation& InTheFrameIntake(SimulatedArena& arena, const std::string& targetedGamePieceType,
                                                  drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width, IntakeSide side,
                                                  int capacity);
        static IntakeSimulation& OverTheBumperIntake(const std::string& targetedGamePieceType, drivesims::AbstractDriveTrainSimulation& driveTrainSimulation,
                                                     units::meter_t width, units::meter_t lengthExtended, IntakeSide side, int capacity);
        static IntakeSimulation& OverTheBumperIntake(SimulatedArena& arena, const std::string& targetedGamePieceType,
                                                     drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width,
                                                     units::meter_t lengthExtended, IntakeSide side, int capacity);

        /** Unregistered until handed to Register, which transfers ownership to the arena. */
        IntakeSimulation(std::string targetedGamePieceType, drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, physics::Shape shape, int capacity);

        static IntakeSimulation& Register(std::unique_ptr<IntakeSimulation> intakeSimulation);
        static IntakeSimulation& Register(SimulatedArena& arena, std::unique_ptr<IntakeSimulation> intakeSimulation);

        void StartIntake();
        void StopIntake();

        [[nodiscard]] int GetGamePiecesAmount() const { return gamePiecesInIntakeCount_; }
        bool ObtainGamePieceFromIntake();
        bool AddGamePieceToIntake();
        bool AddGamePiecesToIntake(int piecesToAdd);
        int SetGamePiecesCount(int gamePiecesInIntakeCount);

        [[nodiscard]] physics::ContactCallback GetGamePieceContactListener();
        void RemoveObtainedGamePieces(SimulatedArena& arena);

        [[nodiscard]] bool IsRunning() const { return intakeRunning_; }
        void SetCustomIntakeCondition(std::function<bool(gamepieces::GamePieceOnFieldSimulation&)> customIntakeCondition);

    private:
        [[nodiscard]] static physics::Shape GetIntakeRectangle(const drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, double width,
                                                               double lengthExtended, IntakeSide side);
        [[nodiscard]] static physics::FixtureMaterial MasslessMaterial();

        const int capacity_;
        int gamePiecesInIntakeCount_ = 0;
        bool intakeRunning_ = false;

        std::deque<gamepieces::GamePieceOnFieldSimulation*> gamePiecesToRemove_;
        drivesims::AbstractDriveTrainSimulation& driveTrainSimulation_;
        const std::string targetedGamePieceType_;
        std::function<bool(gamepieces::GamePieceOnFieldSimulation&)> customIntakeCondition_ = [](gamepieces::GamePieceOnFieldSimulation&) { return true; };
    };
} // namespace maplesim::simulation
