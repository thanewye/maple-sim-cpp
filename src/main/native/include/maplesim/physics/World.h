#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <vector>

#include <units/time.h>

#include "maplesim/physics/Body.h"
#include "maplesim/physics/Fixture.h"

class b2World;

namespace maplesim::physics {
    namespace detail {
        class ContactDispatcher;
    } // namespace detail

    struct WorldSettings {
        int velocityIterations = 6;
        int positionIterations = 2;
    };

    struct Contact {
        Body& bodyA;
        Fixture& fixtureA;
        Body& bodyB;
        Fixture& fixtureB;
    };

    using ContactCallback = std::function<void(const Contact&)>;

    /** Unsubscribes its callback on destruction; safe to outlive the World. */
    class ContactSubscription {
    public:
        struct Registry {
            std::map<std::uint64_t, ContactCallback> callbacks;
            std::uint64_t nextId = 0;
        };

        ContactSubscription() = default;
        ContactSubscription(std::weak_ptr<Registry> registry, std::uint64_t id);
        ~ContactSubscription();

        ContactSubscription(ContactSubscription&& other) noexcept;
        ContactSubscription& operator=(ContactSubscription&& other) noexcept;
        ContactSubscription(const ContactSubscription&) = delete;
        ContactSubscription& operator=(const ContactSubscription&) = delete;

        void Unsubscribe();

    private:
        std::weak_ptr<Registry> registry_;
        std::uint64_t id_ = 0;
    };

    /** Zero-gravity world that integrates applied forces and damping with dyn4j's semantics before each Box2D step. */
    class World {
    public:
        World();
        explicit World(WorldSettings settings);
        ~World();

        World(const World&) = delete;
        World& operator=(const World&) = delete;

        void Step(units::second_t dt);

        void AddBody(Body& body);
        void RemoveBody(Body& body);
        void RemoveAllBodies();
        [[nodiscard]] const std::vector<Body*>& GetBodies() const { return bodies_; }

        /** Fires once when two fixtures begin touching; callbacks run mid-step and must not add or remove bodies or fixtures. */
        [[nodiscard]] ContactSubscription OnBeginContact(ContactCallback callback);

    private:
        WorldSettings settings_;
        std::shared_ptr<ContactSubscription::Registry> beginContactRegistry_;
        std::unique_ptr<detail::ContactDispatcher> contactDispatcher_;
        std::unique_ptr<b2World> b2World_;
        std::vector<Body*> bodies_;
    };
} // namespace maplesim::physics
