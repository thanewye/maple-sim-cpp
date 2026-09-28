#include "pch.h"

#include "maplesim/simulation/drivesims/configs/BoundingCheck.h"

#include <frc/Errors.h>

namespace maplesim::simulation::drivesims::configs::BoundingCheck {
    void Check(double value, double lowerBound, double upperBound, std::string_view variableName, std::string_view unit) {
        if (lowerBound <= value && value <= upperBound) return;
        FRC_ReportError(frc::err::Error, "The provided \"{}\" is {}{}, which seems abnormal, please check its correctness", variableName, value, unit);
    }
} // namespace maplesim::simulation::drivesims::configs::BoundingCheck
