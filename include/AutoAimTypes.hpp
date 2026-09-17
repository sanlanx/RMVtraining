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
    // 数值直接对应历史通信字段 crc。
    NoTarget = 0,
    Stable = 1,
    Unstable = 2,
};

struct GimbalState {
    // 控制器快照：角度单位为弧度，prediction_bias_s 单位为秒。
    float yaw{0.0F};
    float pitch{0.0F};
    float roll{0.0F};
    float prediction_bias_s{0.0F};
};

struct CameraParameters {
    // 所有矩阵均为单通道 CV_64F。刚体变换为
    // p_gimbal = R_camera_to_gimbal * p_camera + t_camera_to_gimbal。
    bool calibrated{false};
    int image_width{0};
    int image_height{0};
    cv::Mat camera_matrix;
    cv::Mat distortion_coefficients;
    cv::Mat rotation_camera_to_gimbal;
    cv::Mat translation_camera_to_gimbal;  // 3x1，单位为米，表示在云台坐标系中的平移。
    double mean_reprojection_error_px{-1.0};

    bool valid() const;
};

struct ArmorDetection {
    // 图像角点按顺时针排列：左上、右上、右下、左下。
    std::array<cv::Point2f, 4> corners{};
    ArmorSize size{ArmorSize::Small};
    int target_id{0};       // 0 保留给未知目标/无目标。
    float confidence{0.0F}; // 分类器置信度，范围为 [0, 1]。
};

struct ArmorPose {
    // 平移向量单位为米。armor_yaw 是云台坐标系中的偏航角（弧度），
    // reprojection_error 是单个观测的重投影误差（像素）。
    ArmorDetection detection;
    cv::Vec3d position_camera_m{0.0, 0.0, 0.0};
    cv::Vec3d position_gimbal_m{0.0, 0.0, 0.0};
    double armor_yaw{0.0};
    double reprojection_error{0.0};
};

struct PredictionResult {
    // EKF 在约定跟踪坐标系中的车体状态：位置/半径单位为米，
    // 速度单位为米每秒，角度单位为弧度，角速度单位为弧度每秒。
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
    // 最终云台指令：距离单位为米，角度单位为弧度，角速度单位为弧度每秒。
    float distance_m{0.0F};
    float target_yaw{0.0F};
    float target_pitch{0.0F};
    float target_yaw_velocity{0.0F};
    float target_pitch_velocity{0.0F};
    bool fire_allowed{false};
};

// 读取 OpenCV YAML，将矩阵统一为 CV_64F，并拒绝占位数据或几何上无效的标定结果。
// 失败时会在 error 中写入错误信息。
bool loadCameraParameters(const std::string& path, CameraParameters& parameters,
                          std::string& error);
