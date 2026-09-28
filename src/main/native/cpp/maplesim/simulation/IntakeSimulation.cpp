#include "pch.h"

#include "maplesim/simulation/IntakeSimulation.h"

#include <algorithm>
#include <utility>

#include <frc/Errors.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>

#include "maplesim/simulation/SimulatedArena.h"

namespace maplesim::simulation {
    IntakeSimulation& IntakeSimulation::InTheFrameIntake(const std::string& targetedGamePieceType,
                                                         drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width, IntakeSide side,
                                                         int capacity) {
        return InTheFrameIntake(SimulatedArena::GetInstance(), targetedGamePieceType, driveTrainSimulation, width, side, capacity);
    }

    IntakeSimulation& IntakeSimulation::InTheFrameIntake(SimulatedArena& arena, const std::string& targetedGamePieceType,
                                                         drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width, IntakeSide side,
                                                         int capacity) {
        return OverTheBumperIntake(arena, targetedGamePieceType, driveTrainSimulation, width, units::meter_t{0.02}, side, capacity);
    }

    IntakeSimulation& IntakeSimulation::OverTheBumperIntake(const std::string& targetedGamePieceType,
                                                            drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width,
                                                            units::meter_t lengthExtended, IntakeSide side, int capacity) {
        return OverTheBumperIntake(SimulatedArena::GetInstance(), targetedGamePieceType, driveTrainSimulation, width, lengthExtended, side, capacity);
    }

    IntakeSimulation& IntakeSimulation::OverTheBumperIntake(SimulatedArena& arena, const std::string& targetedGamePieceType,
                                                            drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, units::meter_t width,
                                                            units::meter_t lengthExtended, IntakeSide side, int capacity) {
        return Register(arena,
                        std::make_unique<IntakeSimulation>(targetedGamePieceType, driveTrainSimulation,
                                                           GetIntakeRectangle(driveTrainSimulation, width.value(), lengthExtended.value(), side), capacity));
    }

    physics::Shape IntakeSimulation::GetIntakeRectangle(const drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, double width,
                                                        double lengthExtended, IntakeSide side) {
        const units::degree_t rotation{side == IntakeSide::kLeft || side == IntakeSide::kRight ? 0.0 : 90.0};
        const double distanceTransformed = lengthExtended / 2 - 0.01;
        const double bumperLengthX = driveTrainSimulation.config.bumperLengthX.value();
        const double bumperWidthY = driveTrainSimulation.config.bumperWidthY.value();
        frc::Translation2d translation;
        switch (side) {
        case IntakeSide::kLeft:
            translation = frc::Translation2d{units::meter_t{0}, units::meter_t{bumperWidthY / 2 + distanceTransformed}};
            break;
        case IntakeSide::kRight:
            translation = frc::Translation2d{units::meter_t{0}, units::meter_t{-bumperWidthY / 2 - distanceTransformed}};
            break;
        case IntakeSide::kFront:
            translation = frc::Translation2d{units::meter_t{bumperLengthX / 2 + distanceTransformed}, units::meter_t{0}};
            break;
        case IntakeSide::kBack:
            translation = frc::Translation2d{units::meter_t{-bumperLengthX / 2 - distanceTransformed / 2}, units::meter_t{0}};
            break;
        }
        return physics::Shape::Rectangle(units::meter_t{width}, units::meter_t{lengthExtended}, frc::Transform2d{translation, frc::Rotation2d{rotation}});
    }

    physics::FixtureMaterial IntakeSimulation::MasslessMaterial() {
        physics::FixtureMaterial material;
        material.density = physics::kilograms_per_square_meter_t{0};
        return material;
    }

    IntakeSimulation::IntakeSimulation(std::string targetedGamePieceType, drivesims::AbstractDriveTrainSimulation& driveTrainSimulation, physics::Shape shape,
                                       int capacity)
        : physics::Fixture(std::move(shape), MasslessMaterial())
        , capacity_(capacity)
        , driveTrainSimulation_(driveTrainSimulation)
        , targetedGamePieceType_(std::move(targetedGamePieceType)) {
        if (capacity > 100) throw FRC_MakeError(frc::err::Error, "capacity too large, max is 100");
    }

    IntakeSimulation& IntakeSimulation::Register(std::unique_ptr<IntakeSimulation> intakeSimulation) {
        return Register(SimulatedArena::GetInstance(), std::move(intakeSimulation));
    }

    IntakeSimulation& IntakeSimulation::Register(SimulatedArena& arena, std::unique_ptr<IntakeSimulation> intakeSimulation) {
        return arena.AddIntakeSimulation(std::move(intakeSimulation));
    }

    void IntakeSimulation::StartIntake() {
        if (intakeRunning_) return;

        driveTrainSimulation_.Attach(*this);
        intakeRunning_ = true;
    }

    void IntakeSimulation::StopIntake() {
        if (!intakeRunning_) return;

        driveTrainSimulation_.Detach(*this);
        intakeRunning_ = false;
    }

    bool IntakeSimulation::ObtainGamePieceFromIntake() {
        if (gamePiecesInIntakeCount_ < 1) return false;
        gamePiecesInIntakeCount_--;
        return true;
    }

    bool IntakeSimulation::AddGamePieceToIntake() {
        const bool toReturn = gamePiecesInIntakeCount_ < capacity_;
        if (toReturn) gamePiecesInIntakeCount_++;

        return toReturn;
    }

    bool IntakeSimulation::AddGamePiecesToIntake(int piecesToAdd) {
        const bool toReturn = gamePiecesInIntakeCount_ + piecesToAdd <= capacity_;
        gamePiecesInIntakeCount_ = std::min(gamePiecesInIntakeCount_ + piecesToAdd, capacity_);
        return toReturn;
    }

    int IntakeSimulation::SetGamePiecesCount(int gamePiecesInIntakeCount) {
        return gamePiecesInIntakeCount_ = std::clamp(gamePiecesInIntakeCount, 0, capacity_);
    }

    void IntakeSimulation::GamePieceContactListener::operator()(const physics::Contact& contact) const {
        if (!intakeSimulation_.intakeRunning_) return;
        if (intakeSimulation_.gamePiecesInIntakeCount_ >= intakeSimulation_.capacity_) return;

        auto* const gamePiece1 = dynamic_cast<gamepieces::GamePieceOnFieldSimulation*>(&contact.bodyA);
        auto* const gamePiece2 = dynamic_cast<gamepieces::GamePieceOnFieldSimulation*>(&contact.bodyB);

        if (gamePiece1 != nullptr && gamePiece1->type == intakeSimulation_.targetedGamePieceType_ && &contact.fixtureB == &intakeSimulation_)
            FlagGamePieceForRemoval(*gamePiece1);
        else if (gamePiece2 != nullptr && gamePiece2->type == intakeSimulation_.targetedGamePieceType_ && &contact.fixtureA == &intakeSimulation_)
            FlagGamePieceForRemoval(*gamePiece2);
    }

    void IntakeSimulation::GamePieceContactListener::FlagGamePieceForRemoval(gamepieces::GamePieceOnFieldSimulation& gamePiece) const {
        if (!intakeSimulation_.customIntakeCondition_(gamePiece)) return;
        intakeSimulation_.gamePiecesToRemove_.push_back(&gamePiece);
        intakeSimulation_.gamePiecesInIntakeCount_++;
    }

    physics::ContactCallback IntakeSimulation::GetGamePieceContactListener() {
        return GamePieceContactListener{*this};
    }

    void IntakeSimulation::RemoveObtainedGamePieces(SimulatedArena& arena) {
        while (!gamePiecesToRemove_.empty()) {
            gamepieces::GamePieceOnFieldSimulation* const gamePiece = gamePiecesToRemove_.front();
            gamePiecesToRemove_.pop_front();
            gamePiece->OnIntake(targetedGamePieceType_);
            arena.RemoveGamePiece(*gamePiece);
        }
    }

    void IntakeSimulation::SetCustomIntakeCondition(std::function<bool(gamepieces::GamePieceOnFieldSimulation&)> customIntakeCondition) {
        customIntakeCondition_ = std::move(customIntakeCondition);
    }
} // namespace maplesim::simulation
