#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <wpi/driverstation/MatchState.hpp>
#include <wpi/driverstation/RobotState.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Pose3d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/nt/BooleanTopic.hpp>
#include <wpi/nt/DoubleTopic.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/system/Timer.hpp>
#include <wpi/units/time.hpp>

#include "maplesim/physics/Body.h"
#include "maplesim/physics/Shape.h"
#include "maplesim/physics/World.h"
#include "maplesim/simulation/drivesims/AbstractDriveTrainSimulation.h"
#include "maplesim/simulation/gamepieces/GamePiece.h"
#include "maplesim/simulation/gamepieces/GamePieceOnFieldSimulation.h"
#include "maplesim/simulation/gamepieces/GamePieceProjectile.h"

namespace maplesim::simulation {
    class IntakeSimulation;

    /** Owns the physics world and every drivetrain, intake, game piece and custom simulation on the field. */
    class SimulatedArena {
    public:
        /** A simulation stepped once per arena sub-tick, after the physics step. */
        class Simulatable {
        public:
            virtual ~Simulatable() = default;
            virtual void SimulationSubTick(int subTickNum) = 0;
        };

        /** Static field obstacles handed to the arena on construction. */
        class FieldMap {
        public:
            virtual ~FieldMap() = default;

        protected:
            void AddBorderLine(const wpi::math::Translation2d& startingPoint, const wpi::math::Translation2d& endingPoint);
            void AddRectangularObstacle(double width, double height, const wpi::math::Pose2d& absolutePositionOnField);
            void AddCustomObstacle(physics::Shape shape, const wpi::math::Pose2d& absolutePositionOnField);

        private:
            friend class SimulatedArena;

            [[nodiscard]] static std::unique_ptr<physics::Body> CreateObstacle(physics::Shape shape);

            std::vector<std::unique_ptr<physics::Body>> obstacles_;
        };

        static inline bool allowCreationOnRealRobot = false;

        virtual ~SimulatedArena();

        SimulatedArena(const SimulatedArena&) = delete;
        SimulatedArena& operator=(const SimulatedArena&) = delete;

        /** Lazily creates an Arena2026Rebuilt; the instance is leaked so its publishers outlive nothing at static teardown. */
        [[nodiscard]] static SimulatedArena& GetInstance();
        /** Destroys the previous instance, aborting if it still owns drivetrains or intakes that callers may reference. */
        static void OverrideInstance(std::unique_ptr<SimulatedArena> newInstance);

        [[nodiscard]] static int GetSimulationSubTicksIn1Period();
        [[nodiscard]] static wpi::units::second_t GetSimulationDt();

        /** Must be called before any simulation is constructed, since caches are sized by the sub-tick count. */
        static void OverrideSimulationTimings(wpi::units::second_t robotPeriod, int simulationSubTicksPerPeriod);

        [[nodiscard]] static wpi::nt::BooleanPublisher& GetResetFieldPublisher();
        [[nodiscard]] static wpi::nt::BooleanSubscriber& GetResetFieldSubscriber();

        [[nodiscard]] int GetScore(bool isBlue) const;
        [[nodiscard]] int GetScore(wpi::Alliance allianceColor) const;
        void AddToScore(bool isBlue, int toAdd);

        template<class T>
        T& AddCustomSimulation(std::unique_ptr<T> simulatable) {
            std::scoped_lock lock{mutex_};
            T& added = *simulatable;
            customSimulations_.push_back(std::move(simulatable));
            return added;
        }

        template<class T>
        T& AddDriveTrainSimulation(std::unique_ptr<T> driveTrainSimulation) {
            std::scoped_lock lock{mutex_};
            T& added = *driveTrainSimulation;
            physicsWorld_.AddBody(added);
            driveTrainSimulations_.push_back(std::move(driveTrainSimulation));
            return added;
        }

        gamepieces::GamePieceOnFieldSimulation& AddGamePiece(std::unique_ptr<gamepieces::GamePieceOnFieldSimulation> gamePiece);
        /** Takes ownership and launches the projectile. */
        gamepieces::GamePieceProjectile& AddGamePieceProjectile(std::unique_ptr<gamepieces::GamePieceProjectile> gamePieceProjectile);

        void EnableBreakdownPublishing();
        void DisableBreakdownPublishing();

        void ReplaceValueInMatchBreakDown(bool isBlueTeam, const std::string& valueKey, double value);
        void ReplaceValueInMatchBreakDown(bool isBlueTeam, const std::string& valueKey, int value);
        void SetupValueForMatchBreakdown(const std::string& valueKey);
        void AddValueToMatchBreakdown(bool isBlueTeam, const std::string& valueKey, double toAdd);
        void AddValueToMatchBreakdown(bool isBlueTeam, const std::string& valueKey, int toAdd);

        /** Removed pieces stay alive until the end of the current sub-tick, so raw pointers from this sub-tick remain valid. */
        bool RemoveGamePiece(gamepieces::GamePieceOnFieldSimulation& gamePiece);
        bool RemovePiece(gamepieces::GamePiece& toRemove);
        bool RemoveProjectile(gamepieces::GamePieceProjectile& gamePieceLaunched);
        void ClearGamePieces();
        void ShutDown();

        void SimulationPeriodic();

        [[nodiscard]] std::vector<gamepieces::GamePieceOnFieldSimulation*> GamePiecesOnField() const;
        [[nodiscard]] std::vector<gamepieces::GamePieceProjectile*> GamePieceLaunched() const;
        [[nodiscard]] virtual std::vector<wpi::math::Pose3d> GetGamePiecesPosesByType(const std::string& type) const;
        [[nodiscard]] std::vector<wpi::math::Pose3d> GetGamePiecesArrayByType(const std::string& type) const;
        [[nodiscard]] std::vector<gamepieces::GamePiece*> GetGamePiecesByType(const std::string& type) const;

        void ResetFieldForAuto();
        virtual void PlaceGamePiecesOnField() = 0;

        std::map<std::string, double> redScoringBreakdown;
        std::map<std::string, double> blueScoringBreakdown;

        std::shared_ptr<wpi::nt::NetworkTable> redTable;
        std::shared_ptr<wpi::nt::NetworkTable> blueTable;
        std::shared_ptr<wpi::nt::NetworkTable> genericInfoTable;
        wpi::nt::DoublePublisher matchClockPublisher;

    protected:
        explicit SimulatedArena(FieldMap&& obstaclesMap);

        virtual void SimulationSubTick(int subTickNum);
        void PublishBreakdown();

        int redScore_ = 0;
        int blueScore_ = 0;
        wpi::Timer matchClock_;
        std::map<std::string, wpi::nt::DoublePublisher> redPublishers_;
        std::map<std::string, wpi::nt::DoublePublisher> bluePublishers_;
        bool shouldPublishMatchBreakdown_ = true;

        IntakeSimulation& AddIntakeSimulation(std::unique_ptr<IntakeSimulation> intakeSimulation);

        physics::World physicsWorld_;
        std::vector<std::unique_ptr<physics::Body>> obstacles_;
        std::vector<std::unique_ptr<drivesims::AbstractDriveTrainSimulation>> driveTrainSimulations_;
        std::vector<std::unique_ptr<IntakeSimulation>> intakeSimulations_;
        std::vector<physics::ContactSubscription> intakeContactSubscriptions_;
        std::vector<std::unique_ptr<gamepieces::GamePiece>> gamePieces_;
        std::vector<std::unique_ptr<gamepieces::GamePiece>> piecesPendingDestruction_;
        std::vector<std::unique_ptr<Simulatable>> customSimulations_;

    private:
        friend class IntakeSimulation;

        bool MovePieceToPendingDestruction(gamepieces::GamePiece& gamePiece);

        mutable std::recursive_mutex mutex_;
    };
} // namespace maplesim::simulation
