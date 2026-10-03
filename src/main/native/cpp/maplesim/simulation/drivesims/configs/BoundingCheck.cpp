#include "pch.h"

#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"

#include <wpi/system/Errors.hpp>

namespace maplesim::simulation::drivesims::configs::BoundingCheck {
    void Check(double value, double lowerBound, double upperBound, std::string_view variableName, std::string_view unit) {
        if (lowerBound <= value && value <= upperBound) return;
        WPILIB_ReportError(wpi::err::Error, "The provided \"{}\" is {}{}, which seems abnormal, please check its correctness", variableName, value, unit);
    }
} // namespace maplesim::simulation::drivesims::configs::BoundingCheck
