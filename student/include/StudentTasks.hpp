#pragma once

#include "AutoAimTypes.hpp"

#include <vector>

#include <opencv2/core.hpp>

// Task 1: manage the camera SDK and return one non-empty CV_8UC3 BGR frame.
// Return false on timeout/disconnect; do not terminate the process from this function.
bool get_pic(cv::Mat& pic);

// Task 2: detect enemy armors and classify their numbers. Every result must
// contain four clockwise image corners, armor size, target ID and confidence.
std::vector<ArmorDetection> armor_detect(const cv::Mat& pic, TeamColor enemy_color);

// Task 3: solve pose with the calibrated intrinsics, then transform observations
// from the camera frame to the gimbal frame. Output positions are in meters.
std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>& detections,
                                  const CameraParameters& camera,
                                  const GimbalState& gimbal);

// Task 4: associate observations, update the rotating-target EKF and predict the
// command. timestamp_seconds is monotonic; prediction_bias_s comes from the controller.
PredictionResult ekf_predict(const std::vector<ArmorPose>& observations,
                             const GimbalState& gimbal, double timestamp_seconds);
