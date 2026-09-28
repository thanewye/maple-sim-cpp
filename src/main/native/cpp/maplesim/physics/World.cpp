#include "pch.h"

#include "maplesim/physics/World.h"

#include <cassert>
#include <utility>

#include <box2d/b2_contact.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>
#include <box2d/b2_world_callbacks.h>

namespace maplesim::physics {
    namespace detail {
        class ContactDispatcher : public b2ContactListener {
        public:
            explicit ContactDispatcher(std::shared_ptr<ContactSubscription::Registry> beginContactRegistry)
                : beginContactRegistry_(std::move(beginContactRegistry)) {}

            void BeginContact(b2Contact* b2ContactHandle) override {
                if (beginContactRegistry_->callbacks.empty()) return;
                Fixture& fixtureA = *reinterpret_cast<Fixture*>(b2ContactHandle->GetFixtureA()->GetUserData().pointer);
                Fixture& fixtureB = *reinterpret_cast<Fixture*>(b2ContactHandle->GetFixtureB()->GetUserData().pointer);
                const Contact contact{*fixtureA.GetBody(), fixtureA, *fixtureB.GetBody(), fixtureB};

                std::vector<std::uint64_t> subscriberIds;
                subscriberIds.reserve(beginContactRegistry_->callbacks.size());
                for (const auto& [id, callback] : beginContactRegistry_->callbacks)
                    subscriberIds.push_back(id);
                for (const std::uint64_t id : subscriberIds) {
                    const auto subscriber = beginContactRegistry_->callbacks.find(id);
                    if (subscriber != beginContactRegistry_->callbacks.end()) subscriber->second(contact);
                }
            }

        private:
            std::shared_ptr<ContactSubscription::Registry> beginContactRegistry_;
        };
    } // namespace detail

    ContactSubscription::ContactSubscription(std::weak_ptr<Registry> registry, std::uint64_t id)
        : registry_(std::move(registry))
        , id_(id) {}

    ContactSubscription::~ContactSubscription() {
        Unsubscribe();
    }

    ContactSubscription::ContactSubscription(ContactSubscription&& other) noexcept
        : registry_(std::exchange(other.registry_, {}))
        , id_(other.id_) {}

    ContactSubscription& ContactSubscription::operator=(ContactSubscription&& other) noexcept {
        if (this != &other) {
            Unsubscribe();
            registry_ = std::exchange(other.registry_, {});
            id_ = other.id_;
        }
        return *this;
    }

    void ContactSubscription::Unsubscribe() {
        if (const std::shared_ptr<Registry> registry = registry_.lock()) registry->callbacks.erase(id_);
        registry_.reset();
    }

    World::World()
        : World(WorldSettings{}) {}

    World::World(WorldSettings settings)
        : settings_(settings)
        , beginContactRegistry_(std::make_shared<ContactSubscription::Registry>())
        , contactDispatcher_(std::make_unique<detail::ContactDispatcher>(beginContactRegistry_))
        , b2World_(std::make_unique<b2World>(b2Vec2{0.0f, 0.0f})) {
        b2World_->SetContactListener(contactDispatcher_.get());
    }

    World::~World() {
        RemoveAllBodies();
    }

    void World::Step(units::second_t dt) {
        for (Body* body : bodies_)
            body->IntegrateAppliedLoads(dt);
        b2World_->Step(static_cast<float>(dt.value()), settings_.velocityIterations, settings_.positionIterations);
    }

    void World::AddBody(Body& body) {
        assert(body.world_ == nullptr);
        assert(!b2World_->IsLocked());
        body.CreateB2Body(*this, *b2World_);
        bodies_.push_back(&body);
    }

    void World::RemoveBody(Body& body) {
        assert(body.world_ == this);
        assert(!b2World_->IsLocked());
        body.DestroyB2Body();
        std::erase(bodies_, &body);
    }

    void World::RemoveAllBodies() {
        assert(!b2World_->IsLocked());
        for (Body* body : bodies_)
            body->DestroyB2Body();
        bodies_.clear();
    }

    ContactSubscription World::OnBeginContact(ContactCallback callback) {
        const std::uint64_t id = beginContactRegistry_->nextId++;
        beginContactRegistry_->callbacks.emplace(id, std::move(callback));
        return ContactSubscription{beginContactRegistry_, id};
    }
} // namespace maplesim::physics
