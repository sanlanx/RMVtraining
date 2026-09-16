#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

enum class TeamColor : std::uint8_t {
    Red = 0,
    Blue = 1,
};

enum class ArmorSize : std::uint8_t {
    Small = 0,
    Large = 1,
};

enum class TrackingState : std::uint16_t {
    // Values map directly to the historical wire field named crc.
    NoTarget = 0,
    Stable = 1,
    Unstable = 2,
};

struct GimbalState {
    // Controller snapshot: angles are rad and prediction_bias_s is seconds.
    float yaw{0.0F};
    float pitch{0.0F};
    float roll{0.0F};
    float prediction_bias_s{0.0F};
};

struct CameraParameters {
    // All matrices are single-channel CV_64F. The rigid transform is
    // p_gimbal = R_camera_to_gimbal * p_camera + t_camera_to_gimbal.
    bool calibrated{false};
    int image_width{0};
    int image_height{0};
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat rotation_camera_to_gimbal;
    cv::Mat translation_camera_to_gimbal;  // 3x1, meters in the gimbal frame.
    double mean_reprojection_error_px{-1.0};

    bool valid() const;
};

struct ArmorDetection {
    // Image corners are clockwise: top-left, top-right, bottom-right, bottom-left.
    std::array<cv::Point2f, 4> corners{};
    ArmorSize size{ArmorSize::Small};
    int target_id{0};       // 0 is reserved for unknown/no target.
    float confidence{0.0F}; // Classifier confidence in [0, 1].
};

struct ArmorPose {
    // Translation vectors use meters. armor_yaw is measured in the gimbal frame
    // in rad, and reprojection_error is the per-observation error in pixels.
    ArmorDetection detection;
    cv::Vec3d position_camera_m{0.0, 0.0, 0.0};
    cv::Vec3d position_gimbal_m{0.0, 0.0, 0.0};
    double armor_yaw{0.0};
    double reprojection_error{0.0};
};

struct PredictionResult {
    // EKF chassis state in the agreed tracking frame: position/radius in m,
    // velocity in m/s, angles in rad and angular velocity in rad/s.
    TrackingState state{TrackingState::NoTarget};
    std::uint8_t target_id{0};
    float center_x_m{0.0F};
    float center_y_m{0.0F};
    float center_velocity_x_m_s{0.0F};
    float center_velocity_y_m_s{0.0F};
    float body_yaw{0.0F};
    float body_yaw_velocity{0.0F};
    float radius_1_m{0.0F};
    float radius_2_m{0.0F};
    // Final gimbal command: distance in m, angles in rad and angular rates in rad/s.
    float distance_m{0.0F};
    float target_yaw{0.0F};
    float target_pitch{0.0F};
    float target_yaw_velocity{0.0F};
    float target_pitch_velocity{0.0F};
    bool fire_allowed{false};
};

// Loads OpenCV YAML, normalizes matrices to CV_64F and rejects placeholder or
// geometrically invalid calibration data. error is populated on failure.
bool loadCameraParameters(const std::string& path, CameraParameters& parameters,
                          std::string& error);
