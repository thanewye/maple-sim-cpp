#pragma once

#include <memory>
#include <vector>

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/moment_of_inertia.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/torque.hpp>

#include "maplesim/physics/Fixture.h"
#include "maplesim/physics/Shape.h"
#include "maplesim/physics/Vector2d.h"

class b2Body;
class b2World;

namespace maplesim::physics {
    class World;

    enum class BodyType { kStatic, kDynamic };

    /** Rigid body whose state lives in the wrapper while detached and in Box2D while added to a World. */
    class Body {
    public:
        Body();
        virtual ~Body();

        Body(const Body&) = delete;
        Body& operator=(const Body&) = delete;

        void SetBodyType(BodyType type);
        [[nodiscard]] BodyType GetBodyType() const { return type_; }

        /** Creates a fixture owned by this body. */
        Fixture& AddFixture(Shape shape, FixtureMaterial material);
        /** Attaches a fixture owned by the caller; it detaches itself on destruction. */
        void Attach(Fixture& fixture);
        /** Detaches the fixture; body-owned fixtures are destroyed. */
        void Detach(Fixture& fixture);
        [[nodiscard]] const std::vector<Fixture*>& GetFixtures() const { return fixtures_; }

        [[nodiscard]] wpi::math::Pose2d GetPose() const;
        void SetPose(const wpi::math::Pose2d& pose);

        [[nodiscard]] LinearVelocity2d GetLinearVelocity() const;
        void SetLinearVelocity(const LinearVelocity2d& velocity);
        [[nodiscard]] wpi::units::radians_per_second_t GetAngularVelocity() const;
        void SetAngularVelocity(wpi::units::radians_per_second_t velocity);
        /** Field-relative linear and angular velocity. */
        [[nodiscard]] wpi::math::ChassisVelocities GetVelocity() const;
        void SetVelocity(const wpi::math::ChassisVelocities& fieldRelativeVelocity);

        [[nodiscard]] wpi::math::Translation2d GetWorldCenter() const;
        [[nodiscard]] wpi::math::Translation2d GetWorldPoint(const wpi::math::Translation2d& localPoint) const;
        [[nodiscard]] LinearVelocity2d GetVelocityAtPoint(const wpi::math::Translation2d& worldPoint) const;

        /** Applies a force at the center of mass until the next World step. */
        void ApplyForce(const Force2d& force);
        /** Applies a force at a world point until the next World step, producing torque about the current world center. */
        void ApplyForce(const Force2d& force, const wpi::math::Translation2d& worldPoint);
        void ApplyTorque(wpi::units::newton_meter_t torque);

        [[nodiscard]] wpi::units::kilogram_t GetMass() const;
        /** Moment of inertia about the center of mass. */
        [[nodiscard]] wpi::units::kilogram_square_meter_t GetMomentOfInertia() const;

        void SetLinearDamping(double damping) { linearDamping_ = damping; }
        [[nodiscard]] double GetLinearDamping() const { return linearDamping_; }
        void SetAngularDamping(double damping) { angularDamping_ = damping; }
        [[nodiscard]] double GetAngularDamping() const { return angularDamping_; }
        void SetBullet(bool bullet);
        [[nodiscard]] bool IsBullet() const { return bullet_; }

        [[nodiscard]] World* GetWorld() const { return world_; }

    private:
        friend class World;

        struct MassProperties {
            wpi::units::kilogram_t mass{0};
            wpi::units::kilogram_square_meter_t inertiaAboutCenter{0};
            wpi::math::Translation2d localCenter;
        };

        [[nodiscard]] MassProperties ComputeMassProperties() const;
        void CreateB2Fixture(Fixture& fixture);
        void CreateB2Body(World& world, b2World& b2WorldHandle);
        void DestroyB2Body();
        void IntegrateAppliedLoads(wpi::units::second_t dt);

        BodyType type_ = BodyType::kStatic;
        wpi::math::Pose2d pose_;
        LinearVelocity2d linearVelocity_;
        wpi::units::radians_per_second_t angularVelocity_{0};
        double linearDamping_ = 0.0;
        double angularDamping_ = 0.01;
        bool bullet_ = false;

        Force2d appliedForce_;
        wpi::units::newton_meter_t appliedTorque_{0};

        std::vector<Fixture*> fixtures_;
        std::vector<std::unique_ptr<Fixture>> ownedFixtures_;

        World* world_ = nullptr;
        b2Body* b2Body_ = nullptr;
    };
} // namespace maplesim::physics
