#pragma once

#include <string_view>

namespace maplesim::simulation::drivesims::configs::BoundingCheck {
    /** Reports an error when the value falls outside the bounds; never throws. */
    void Check(double value, double lowerBound, double upperBound, std::string_view variableName, std::string_view unit);
} // namespace maplesim::simulation::drivesims::configs::BoundingCheck
