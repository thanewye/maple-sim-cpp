#include "pch.h"

#include "maplesim/physics/Fixture.h"

#include <utility>

#include "maplesim/physics/Body.h"

namespace maplesim::physics {
    Fixture::Fixture(Shape shape, FixtureMaterial material)
        : shape_(std::move(shape))
        , material_(material) {}

    Fixture::~Fixture() {
        if (body_ != nullptr) body_->Detach(*this);
    }
} // namespace maplesim::physics
