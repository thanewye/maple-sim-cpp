#include "pch.h"

#include "maplesim/physics/Body.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>

#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>

#include "maplesim/physics/World.h"
#include "maplesim/physics/detail/Box2dConversions.h"

namespace maplesim::physics {
    namespace {
        [[nodiscard]] b2BodyType ToB2BodyType(BodyType type) {
            return type == BodyType::kDynamic ? b2_dynamicBody : b2_staticBody;
        }

        [[nodiscard]] double DampingScale(double damping, double dtSeconds) {
            return std::clamp(1.0 - dtSeconds * damping, 0.0, 1.0);
        }
    } // namespace

    Body::Body() = default;

    Body::~Body() {
        if (world_ != nullptr) world_->RemoveBody(*this);
        for (Fixture* fixture : fixtures_)
            fixture->body_ = nullptr;
    }

    void Body::SetBodyType(BodyType type) {
        type_ = type;
        if (b2Body_ != nullptr) b2Body_->SetType(ToB2BodyType(type));
    }

    Fixture& Body::AddFixture(Shape shape, FixtureMaterial material) {
        Fixture& fixture = *ownedFixtures_.emplace_back(std::make_unique<Fixture>(std::move(shape), material));
        Attach(fixture);
        return fixture;
    }

    void Body::Attach(Fixture& fixture) {
        assert(fixture.body_ == nullptr);
        fixture.body_ = this;
        fixtures_.push_back(&fixture);
        if (b2Body_ != nullptr) CreateB2Fixture(fixture);
    }

    void Body::Detach(Fixture& fixture) {
        assert(fixture.body_ == this);
        if (fixture.b2Fixture_ != nullptr) b2Body_->DestroyFixture(fixture.b2Fixture_);
        fixture.b2Fixture_ = nullptr;
        fixture.body_ = nullptr;
        std::erase(fixtures_, &fixture);
        std::erase_if(ownedFixtures_, [&fixture](const std::unique_ptr<Fixture>& owned) { return owned.get() == &fixture; });
    }

    wpi::math::Pose2d Body::GetPose() const {
        if (b2Body_ == nullptr) return pose_;
        return wpi::math::Pose2d{detail::ToTranslation2d(b2Body_->GetPosition()), wpi::math::Rotation2d{wpi::units::radian_t{b2Body_->GetAngle()}}};
    }

    void Body::SetPose(const wpi::math::Pose2d& pose) {
        pose_ = pose;
        if (b2Body_ != nullptr) b2Body_->SetTransform(detail::ToB2Vec2(pose.Translation()), static_cast<float>(pose.Rotation().Radians().value()));
    }

    LinearVelocity2d Body::GetLinearVelocity() const {
        if (b2Body_ == nullptr) return linearVelocity_;
        const b2Vec2& velocity = b2Body_->GetLinearVelocity();
        return LinearVelocity2d{wpi::units::meters_per_second_t{velocity.x}, wpi::units::meters_per_second_t{velocity.y}};
    }

    void Body::SetLinearVelocity(const LinearVelocity2d& velocity) {
        linearVelocity_ = velocity;
        if (b2Body_ != nullptr) b2Body_->SetLinearVelocity(b2Vec2{static_cast<float>(velocity.x.value()), static_cast<float>(velocity.y.value())});
    }

    wpi::units::radians_per_second_t Body::GetAngularVelocity() const {
        if (b2Body_ == nullptr) return angularVelocity_;
        return wpi::units::radians_per_second_t{b2Body_->GetAngularVelocity()};
    }

    void Body::SetAngularVelocity(wpi::units::radians_per_second_t velocity) {
        angularVelocity_ = velocity;
        if (b2Body_ != nullptr) b2Body_->SetAngularVelocity(static_cast<float>(velocity.value()));
    }

    wpi::math::ChassisVelocities Body::GetVelocity() const {
        const LinearVelocity2d linearVelocity = GetLinearVelocity();
        return wpi::math::ChassisVelocities{linearVelocity.x, linearVelocity.y, GetAngularVelocity()};
    }

    void Body::SetVelocity(const wpi::math::ChassisVelocities& fieldRelativeVelocity) {
        SetLinearVelocity(LinearVelocity2d{fieldRelativeVelocity.vx, fieldRelativeVelocity.vy});
        SetAngularVelocity(fieldRelativeVelocity.omega);
    }

    wpi::math::Translation2d Body::GetWorldCenter() const {
        if (b2Body_ != nullptr) return detail::ToTranslation2d(b2Body_->GetWorldCenter());
        return GetWorldPoint(ComputeMassProperties().localCenter);
    }

    wpi::math::Translation2d Body::GetWorldPoint(const wpi::math::Translation2d& localPoint) const {
        const wpi::math::Pose2d pose = GetPose();
        return pose.Translation() + localPoint.RotateBy(pose.Rotation());
    }

    LinearVelocity2d Body::GetVelocityAtPoint(const wpi::math::Translation2d& worldPoint) const {
        const wpi::math::Translation2d leverArm = worldPoint - GetWorldCenter();
        const double angularVelocity = GetAngularVelocity().value();
        const LinearVelocity2d tangentialVelocity{
            wpi::units::meters_per_second_t{-angularVelocity * leverArm.Y().value()},
            wpi::units::meters_per_second_t{angularVelocity * leverArm.X().value()},
        };
        return GetLinearVelocity() + tangentialVelocity;
    }

    void Body::ApplyForce(const Force2d& force) {
        appliedForce_ += force;
    }

    void Body::ApplyForce(const Force2d& force, const wpi::math::Translation2d& worldPoint) {
        const wpi::math::Translation2d leverArm = worldPoint - GetWorldCenter();
        appliedForce_ += force;
        appliedTorque_ += wpi::units::newton_meter_t{leverArm.X().value() * force.y.value() - leverArm.Y().value() * force.x.value()};
    }

    void Body::ApplyTorque(wpi::units::newton_meter_t torque) {
        appliedTorque_ += torque;
    }

    wpi::units::kilogram_t Body::GetMass() const {
        if (b2Body_ != nullptr) return wpi::units::kilogram_t{b2Body_->GetMass()};
        return ComputeMassProperties().mass;
    }

    wpi::units::kilogram_square_meter_t Body::GetMomentOfInertia() const {
        if (b2Body_ == nullptr) return ComputeMassProperties().inertiaAboutCenter;
        const b2Vec2 localCenter = b2Body_->GetLocalCenter();
        return wpi::units::kilogram_square_meter_t{b2Body_->GetInertia() - b2Body_->GetMass() * b2Dot(localCenter, localCenter)};
    }

    void Body::SetBullet(bool bullet) {
        bullet_ = bullet;
        if (b2Body_ != nullptr) b2Body_->SetBullet(bullet);
    }

    Body::MassProperties Body::ComputeMassProperties() const {
        float mass = 0.0f;
        float inertiaAboutOrigin = 0.0f;
        b2Vec2 weightedCenter{0.0f, 0.0f};
        for (const Fixture* fixture : fixtures_) {
            const float density = static_cast<float>(fixture->GetMaterial().density.value());
            if (density <= 0.0f) continue;
            detail::VisitB2Shape(fixture->GetShape(), [&](const b2Shape& shape) {
                b2MassData massData;
                shape.ComputeMass(&massData, density);
                mass += massData.mass;
                weightedCenter += massData.mass * massData.center;
                inertiaAboutOrigin += massData.I;
            });
        }
        const b2Vec2 localCenter = mass > 0.0f ? (1.0f / mass) * weightedCenter : b2Vec2{0.0f, 0.0f};
        MassProperties properties;
        properties.mass = wpi::units::kilogram_t{mass};
        properties.inertiaAboutCenter = wpi::units::kilogram_square_meter_t{inertiaAboutOrigin - mass * b2Dot(localCenter, localCenter)};
        properties.localCenter = detail::ToTranslation2d(localCenter);
        return properties;
    }

    void Body::CreateB2Fixture(Fixture& fixture) {
        const FixtureMaterial& material = fixture.GetMaterial();
        detail::VisitB2Shape(fixture.GetShape(), [&](const b2Shape& shape) {
            b2FixtureDef definition;
            definition.shape = &shape;
            definition.friction = static_cast<float>(material.friction);
            definition.restitution = static_cast<float>(material.restitution);
            definition.restitutionThreshold = static_cast<float>(material.restitutionThreshold.value());
            definition.density = static_cast<float>(material.density.value());
            definition.userData.pointer = reinterpret_cast<std::uintptr_t>(&fixture);
            fixture.b2Fixture_ = b2Body_->CreateFixture(&definition);
        });
    }

    void Body::CreateB2Body(World& world, b2World& b2WorldHandle) {
        b2BodyDef definition;
        definition.type = ToB2BodyType(type_);
        definition.position = detail::ToB2Vec2(pose_.Translation());
        definition.angle = static_cast<float>(pose_.Rotation().Radians().value());
        definition.bullet = bullet_;
        definition.userData.pointer = reinterpret_cast<std::uintptr_t>(this);
        b2Body_ = b2WorldHandle.CreateBody(&definition);
        world_ = &world;
        for (Fixture* fixture : fixtures_)
            CreateB2Fixture(*fixture);
        SetLinearVelocity(linearVelocity_);
        SetAngularVelocity(angularVelocity_);
    }

    void Body::DestroyB2Body() {
        pose_ = GetPose();
        linearVelocity_ = GetLinearVelocity();
        angularVelocity_ = GetAngularVelocity();
        for (Fixture* fixture : fixtures_)
            fixture->b2Fixture_ = nullptr;
        b2Body_->GetWorld()->DestroyBody(b2Body_);
        b2Body_ = nullptr;
        world_ = nullptr;
    }

    void Body::IntegrateAppliedLoads(wpi::units::second_t dt) {
        const Force2d force = std::exchange(appliedForce_, Force2d{});
        const wpi::units::newton_meter_t torque = std::exchange(appliedTorque_, wpi::units::newton_meter_t{0});
        if (b2Body_->GetType() != b2_dynamicBody) return;
        if (force != Force2d{} || torque != wpi::units::newton_meter_t{0}) b2Body_->SetAwake(true);
        if (!b2Body_->IsAwake()) return;

        const double dtSeconds = dt.value();
        const double mass = GetMass().value();
        const double inertia = GetMomentOfInertia().value();
        double velocityX = b2Body_->GetLinearVelocity().x;
        double velocityY = b2Body_->GetLinearVelocity().y;
        double angularVelocity = b2Body_->GetAngularVelocity();
        if (mass > 0.0) {
            velocityX += dtSeconds * force.x.value() / mass;
            velocityY += dtSeconds * force.y.value() / mass;
        }
        if (inertia > 0.0) angularVelocity += dtSeconds * torque.value() / inertia;
        const double linearScale = DampingScale(linearDamping_, dtSeconds);
        const double angularScale = DampingScale(angularDamping_, dtSeconds);
        b2Body_->SetLinearVelocity(b2Vec2{static_cast<float>(velocityX * linearScale), static_cast<float>(velocityY * linearScale)});
        b2Body_->SetAngularVelocity(static_cast<float>(angularVelocity * angularScale));
    }
} // namespace maplesim::physics
