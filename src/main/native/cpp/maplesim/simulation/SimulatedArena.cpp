#include "pch.h"

#include "maplesim/simulation/SimulatedArena.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <string_view>

#include <frc/Errors.h>
#include <frc/RobotBase.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>

#include "maplesim/physics/Fixture.h"
#include "maplesim/simulation/IntakeSimulation.h"
#include "maplesim/simulation/motorsims/SimulatedBattery.h"
#include "maplesim/simulation/seasonspecific/rebuilt2026/Arena2026Rebuilt.h"

namespace maplesim::simulation {
    namespace {
        constexpr int kDefaultSimulationSubTicksIn1Period = 5;

        struct SimulationTimings {
            int simulationSubTicksIn1Period = kDefaultSimulationSubTicksIn1Period;
            units::second_t simulationDt = frc::TimedRobot::kDefaultPeriod / kDefaultSimulationSubTicksIn1Period;
        };

        [[nodiscard]] SimulationTimings& GetSimulationTimings() {
            static SimulationTimings timings;
            return timings;
        }

        [[nodiscard]] std::mutex& GetOverrideTimingsMutex() {
            static std::mutex mutex;
            return mutex;
        }

        [[nodiscard]] std::recursive_mutex& GetSimulationPeriodicClassMutex() {
            static std::recursive_mutex mutex;
            return mutex;
        }

        [[nodiscard]] SimulatedArena*& GetInstanceSlot() {
            static SimulatedArena* instance = nullptr;
            return instance;
        }

        struct ResetFieldTopic {
            nt::BooleanPublisher publisher;
            nt::BooleanSubscriber subscriber;
        };

        [[nodiscard]] ResetFieldTopic& GetResetFieldTopic() {
            static ResetFieldTopic* const topic = [] {
                nt::BooleanTopic booleanTopic =
                    nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData")->GetBooleanTopic("Reset Field");
                return new ResetFieldTopic{booleanTopic.Publish(), booleanTopic.Subscribe(false)};
            }();
            return *topic;
        }

        [[noreturn]] void FailOverrideInstance(std::string_view message) {
            FRC_ReportError(frc::err::Error, "[MapleSim] {}", message);
            std::fprintf(stderr, "[MapleSim] %.*s\n", static_cast<int>(message.size()), message.data());
            std::fflush(stderr);
            std::abort();
        }
    } // namespace

    SimulatedArena& SimulatedArena::GetInstance() {
        if (frc::RobotBase::IsReal() && !allowCreationOnRealRobot)
            throw FRC_MakeError(frc::err::Error, "MapleSim is running on a real robot! (If you would actually want that, set "
                                                 "SimulatedArena::allowCreationOnRealRobot to true).");

        SimulatedArena*& instance = GetInstanceSlot();
        if (instance == nullptr) instance = new seasonspecific::rebuilt2026::Arena2026Rebuilt();

        return *instance;
    }

    void SimulatedArena::OverrideInstance(std::unique_ptr<SimulatedArena> newInstance) {
        SimulatedArena*& instance = GetInstanceSlot();
        if (instance != nullptr) {
            if (!instance->driveTrainSimulations_.empty() || !instance->intakeSimulations_.empty())
                FailOverrideInstance("SimulatedArena::OverrideInstance called while the previous arena still owns drivetrains or intakes, which would "
                                     "leave their references dangling. Override the arena before adding any simulations to it.");
            delete instance;
            instance = new seasonspecific::rebuilt2026::Arena2026Rebuilt();
            delete instance;
        }
        instance = newInstance.release();
    }

    int SimulatedArena::GetSimulationSubTicksIn1Period() {
        return GetSimulationTimings().simulationSubTicksIn1Period;
    }

    units::second_t SimulatedArena::GetSimulationDt() {
        return GetSimulationTimings().simulationDt;
    }

    void SimulatedArena::OverrideSimulationTimings(units::second_t robotPeriod, int simulationSubTicksPerPeriod) {
        std::scoped_lock lock{GetOverrideTimingsMutex()};
        SimulationTimings& timings = GetSimulationTimings();
        timings.simulationSubTicksIn1Period = simulationSubTicksPerPeriod;
        timings.simulationDt = robotPeriod / timings.simulationSubTicksIn1Period;
    }

    nt::BooleanPublisher& SimulatedArena::GetResetFieldPublisher() {
        return GetResetFieldTopic().publisher;
    }

    nt::BooleanSubscriber& SimulatedArena::GetResetFieldSubscriber() {
        return GetResetFieldTopic().subscriber;
    }

    SimulatedArena::SimulatedArena(FieldMap&& obstaclesMap)
        : redTable(nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown/Red Alliance"))
        , blueTable(nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown/blue Alliance"))
        , genericInfoTable(nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard/MapleSim/MatchData/Breakdown"))
        , matchClockPublisher(genericInfoTable->GetDoubleTopic("Match Clock").Publish())
        , obstacles_(std::move(obstaclesMap.obstacles_)) {
        for (const std::unique_ptr<physics::Body>& obstacle : obstacles_)
            physicsWorld_.AddBody(*obstacle);
        SetupValueForMatchBreakdown("TotalScore");
        SetupValueForMatchBreakdown("TeleopScore");
        SetupValueForMatchBreakdown("Auto/AutoScore");
        GetResetFieldPublisher().Set(false);
        matchClock_.Start();
    }

    SimulatedArena::~SimulatedArena() = default;

    int SimulatedArena::GetScore(bool isBlue) const {
        return isBlue ? blueScore_ : redScore_;
    }

    int SimulatedArena::GetScore(frc::DriverStation::Alliance allianceColor) const {
        return GetScore(allianceColor == frc::DriverStation::Alliance::kBlue);
    }

    void SimulatedArena::AddToScore(bool isBlue, int toAdd) {
        if (isBlue) blueScore_ += toAdd;
        else redScore_ += toAdd;
        AddValueToMatchBreakdown(isBlue, frc::DriverStation::IsAutonomous() ? "Auto/AutoScore" : "TeleopScore", toAdd);
    }

    IntakeSimulation& SimulatedArena::AddIntakeSimulation(std::unique_ptr<IntakeSimulation> intakeSimulation) {
        std::scoped_lock lock{mutex_};
        IntakeSimulation& added = *intakeSimulation;
        intakeSimulations_.push_back(std::move(intakeSimulation));
        intakeContactSubscriptions_.push_back(physicsWorld_.OnBeginContact(added.GetGamePieceContactListener()));
        return added;
    }

    gamepieces::GamePieceOnFieldSimulation& SimulatedArena::AddGamePiece(std::unique_ptr<gamepieces::GamePieceOnFieldSimulation> gamePiece) {
        std::scoped_lock lock{mutex_};
        gamepieces::GamePieceOnFieldSimulation& added = *gamePiece;
        physicsWorld_.AddBody(added);
        gamePieces_.push_back(std::move(gamePiece));
        return added;
    }

    gamepieces::GamePieceProjectile& SimulatedArena::AddGamePieceProjectile(std::unique_ptr<gamepieces::GamePieceProjectile> gamePieceProjectile) {
        std::scoped_lock lock{mutex_};
        gamepieces::GamePieceProjectile& added = *gamePieceProjectile;
        gamePieces_.push_back(std::move(gamePieceProjectile));
        added.Launch();
        return added;
    }

    void SimulatedArena::EnableBreakdownPublishing() {
        shouldPublishMatchBreakdown_ = true;
    }

    void SimulatedArena::DisableBreakdownPublishing() {
        shouldPublishMatchBreakdown_ = false;
    }

    void SimulatedArena::PublishBreakdown() {
        for (const auto& [key, value] : redScoringBreakdown) {
            auto publisher = redPublishers_.find(key);
            if (publisher == redPublishers_.end()) publisher = redPublishers_.emplace(key, redTable->GetDoubleTopic(key).Publish()).first;
            publisher->second.Set(value);
        }
        for (const auto& [key, value] : blueScoringBreakdown) {
            auto publisher = bluePublishers_.find(key);
            if (publisher == bluePublishers_.end()) publisher = bluePublishers_.emplace(key, blueTable->GetDoubleTopic(key).Publish()).first;
            publisher->second.Set(value);
        }
    }

    void SimulatedArena::ReplaceValueInMatchBreakDown(bool isBlueTeam, const std::string& valueKey, double value) {
        if (isBlueTeam) blueScoringBreakdown[valueKey] = value;
        else redScoringBreakdown[valueKey] = value;
    }

    void SimulatedArena::ReplaceValueInMatchBreakDown(bool isBlueTeam, const std::string& valueKey, int value) {
        ReplaceValueInMatchBreakDown(isBlueTeam, valueKey, static_cast<double>(value));
    }

    void SimulatedArena::SetupValueForMatchBreakdown(const std::string& valueKey) {
        ReplaceValueInMatchBreakDown(true, valueKey, 0);
        ReplaceValueInMatchBreakDown(false, valueKey, 0);
    }

    void SimulatedArena::AddValueToMatchBreakdown(bool isBlueTeam, const std::string& valueKey, double toAdd) {
        if (isBlueTeam) blueScoringBreakdown[valueKey] += toAdd;
        else redScoringBreakdown[valueKey] += toAdd;
    }

    void SimulatedArena::AddValueToMatchBreakdown(bool isBlueTeam, const std::string& valueKey, int toAdd) {
        AddValueToMatchBreakdown(isBlueTeam, valueKey, static_cast<double>(toAdd));
    }

    bool SimulatedArena::RemoveGamePiece(gamepieces::GamePieceOnFieldSimulation& gamePiece) {
        std::scoped_lock lock{mutex_};
        if (gamePiece.GetWorld() == &physicsWorld_) physicsWorld_.RemoveBody(gamePiece);
        return MovePieceToPendingDestruction(gamePiece);
    }

    bool SimulatedArena::RemovePiece(gamepieces::GamePiece& toRemove) {
        std::scoped_lock lock{mutex_};
        if (toRemove.IsGrounded()) return RemoveGamePiece(static_cast<gamepieces::GamePieceOnFieldSimulation&>(toRemove));
        return RemoveProjectile(static_cast<gamepieces::GamePieceProjectile&>(toRemove));
    }

    bool SimulatedArena::RemoveProjectile(gamepieces::GamePieceProjectile& gamePieceLaunched) {
        std::scoped_lock lock{mutex_};
        return MovePieceToPendingDestruction(gamePieceLaunched);
    }

    bool SimulatedArena::MovePieceToPendingDestruction(gamepieces::GamePiece& gamePiece) {
        const auto found =
            std::ranges::find_if(gamePieces_, [&gamePiece](const std::unique_ptr<gamepieces::GamePiece>& owned) { return owned.get() == &gamePiece; });
        if (found == gamePieces_.end()) return false;
        piecesPendingDestruction_.push_back(std::move(*found));
        gamePieces_.erase(found);
        return true;
    }

    void SimulatedArena::ClearGamePieces() {
        std::scoped_lock lock{mutex_};
        for (gamepieces::GamePieceOnFieldSimulation* gamePiece : GamePiecesOnField())
            if (gamePiece->GetWorld() == &physicsWorld_) physicsWorld_.RemoveBody(*gamePiece);
        std::ranges::move(gamePieces_, std::back_inserter(piecesPendingDestruction_));
        gamePieces_.clear();
        blueScore_ = 0;
        redScore_ = 0;
    }

    void SimulatedArena::ShutDown() {
        std::scoped_lock lock{mutex_};
        physicsWorld_.RemoveAllBodies();
    }

    void SimulatedArena::SimulationPeriodic() {
        std::scoped_lock lock{mutex_};
        std::scoped_lock classLock{GetSimulationPeriodicClassMutex()};
        const auto t0 = std::chrono::steady_clock::now();

        for (int i = 0; i < GetSimulationSubTicksIn1Period(); i++)
            SimulationSubTick(i);

        frc::SmartDashboard::PutNumber("MapleArenaSimulation/Dyn4jEngineCPUTimeMS",
                                       std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());

        if (GetResetFieldSubscriber().Get()) {
            ResetFieldForAuto();
            GetResetFieldPublisher().Set(false);
        }
    }

    void SimulatedArena::SimulationSubTick(int subTickNum) {
        motorsims::SimulatedBattery::SimulationSubTick();
        for (const std::unique_ptr<drivesims::AbstractDriveTrainSimulation>& driveTrainSimulation : driveTrainSimulations_)
            driveTrainSimulation->SimulationSubTick();

        gamepieces::GamePieceProjectile::UpdateGamePieceProjectiles(*this, GamePieceLaunched());

        physicsWorld_.Step(GetSimulationDt());

        for (const std::unique_ptr<IntakeSimulation>& intake : intakeSimulations_)
            intake->RemoveObtainedGamePieces(*this);
        for (std::size_t i = 0; i < customSimulations_.size(); i++)
            customSimulations_[i]->SimulationSubTick(subTickNum);

        ReplaceValueInMatchBreakDown(true, "TotalScore", blueScore_);
        ReplaceValueInMatchBreakDown(false, "TotalScore", redScore_);

        if (shouldPublishMatchBreakdown_) {
            PublishBreakdown();
            matchClockPublisher.Set(matchClock_.Get().value());
        }

        piecesPendingDestruction_.clear();
    }

    std::vector<gamepieces::GamePieceOnFieldSimulation*> SimulatedArena::GamePiecesOnField() const {
        std::scoped_lock lock{mutex_};
        std::vector<gamepieces::GamePieceOnFieldSimulation*> returnList;
        for (const std::unique_ptr<gamepieces::GamePiece>& gamePiece : gamePieces_)
            if (gamePiece->IsGrounded()) returnList.push_back(static_cast<gamepieces::GamePieceOnFieldSimulation*>(gamePiece.get()));
        return returnList;
    }

    std::vector<gamepieces::GamePieceProjectile*> SimulatedArena::GamePieceLaunched() const {
        std::scoped_lock lock{mutex_};
        std::vector<gamepieces::GamePieceProjectile*> returnList;
        for (const std::unique_ptr<gamepieces::GamePiece>& gamePiece : gamePieces_)
            if (!gamePiece->IsGrounded()) returnList.push_back(static_cast<gamepieces::GamePieceProjectile*>(gamePiece.get()));
        return returnList;
    }

    std::vector<frc::Pose3d> SimulatedArena::GetGamePiecesPosesByType(const std::string& type) const {
        std::scoped_lock lock{mutex_};
        std::vector<frc::Pose3d> gamePiecesPoses;
        for (const std::unique_ptr<gamepieces::GamePiece>& gamePiece : gamePieces_)
            if (gamePiece->GetType() == type) gamePiecesPoses.push_back(gamePiece->GetPose3d());
        return gamePiecesPoses;
    }

    std::vector<frc::Pose3d> SimulatedArena::GetGamePiecesArrayByType(const std::string& type) const {
        return GetGamePiecesPosesByType(type);
    }

    std::vector<gamepieces::GamePiece*> SimulatedArena::GetGamePiecesByType(const std::string& type) const {
        std::scoped_lock lock{mutex_};
        std::vector<gamepieces::GamePiece*> gamePiecesOfType;
        for (const std::unique_ptr<gamepieces::GamePiece>& gamePiece : gamePieces_)
            if (gamePiece->GetType() == type) gamePiecesOfType.push_back(gamePiece.get());
        return gamePiecesOfType;
    }

    void SimulatedArena::ResetFieldForAuto() {
        std::scoped_lock lock{mutex_};
        ClearGamePieces();
        matchClock_.Reset();
        PlaceGamePiecesOnField();
    }

    void SimulatedArena::FieldMap::AddBorderLine(const frc::Translation2d& startingPoint, const frc::Translation2d& endingPoint) {
        AddCustomObstacle(physics::Shape::Segment(startingPoint, endingPoint), frc::Pose2d{});
    }

    void SimulatedArena::FieldMap::AddRectangularObstacle(double width, double height, const frc::Pose2d& absolutePositionOnField) {
        AddCustomObstacle(physics::Shape::Rectangle(units::meter_t{width}, units::meter_t{height}), absolutePositionOnField);
    }

    void SimulatedArena::FieldMap::AddCustomObstacle(physics::Shape shape, const frc::Pose2d& absolutePositionOnField) {
        std::unique_ptr<physics::Body> obstacle = CreateObstacle(std::move(shape));
        obstacle->SetPose(absolutePositionOnField);
        obstacles_.push_back(std::move(obstacle));
    }

    std::unique_ptr<physics::Body> SimulatedArena::FieldMap::CreateObstacle(physics::Shape shape) {
        auto obstacle = std::make_unique<physics::Body>();
        obstacle->SetBodyType(physics::BodyType::kStatic);
        physics::FixtureMaterial material;
        material.friction = 0.6;
        material.restitution = 0.3;
        obstacle->AddFixture(std::move(shape), material);
        return obstacle;
    }
} // namespace maplesim::simulation
