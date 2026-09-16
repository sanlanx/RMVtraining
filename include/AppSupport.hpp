#pragma once

#include "AutoAimTypes.hpp"
#include "Communication.hpp"

#include <cstdint>

// Only status 0 (our team red) and 5 (our team blue) belong to this assignment.
bool isAutoAimStatus(std::uint8_t status) noexcept;
// status encodes our team color; detection needs the opposite (enemy) color.
TeamColor enemyColorFromStatus(std::uint8_t status) noexcept;
// Extracts controller-owned attitude and prediction timing fields.
GimbalState gimbalStateFromMessage(const MessData_AutoAim& message) noexcept;

// Produces a deterministic no-target frame and clears every fire-related field.
void setSafeOutput(Translator& translator) noexcept;
// Validates an EKF result and maps SI units to the unchanged wire protocol.
// In particular, center_x/y are converted from meters to millimeters.
bool applyPrediction(Translator& translator, const PredictionResult& prediction) noexcept;
