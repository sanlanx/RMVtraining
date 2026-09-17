#include "AppSupport.hpp"

#include <cassert>
#include <cmath>
#include <limits>

int main() {
    static_assert(sizeof(Translator) == 64, "The serial message layout must remain unchanged");

    // 标定参数应接受文档规定的矩阵形状，并拒绝外形相似但不合法的输入。
    CameraParameters camera;
    assert(!camera.valid());
    camera.calibrated = true;
    camera.image_width = 1440;
    camera.image_height = 1080;
    camera.camera_matrix =
        (cv::Mat_<double>(3, 3) << 1200.0, 0.0, 720.0, 0.0, 1200.0, 540.0, 0.0,
         0.0, 1.0);
    camera.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
    camera.rotation_camera_to_gimbal = cv::Mat::eye(3, 3, CV_64F);
    camera.translation_camera_to_gimbal = cv::Mat::zeros(3, 1, CV_64F);
    camera.mean_reprojection_error_px = 0.2;
    assert(camera.valid());

    camera.distortion_coefficients = cv::Mat::zeros(3, 3, CV_64F);
    assert(!camera.valid());
    camera.distortion_coefficients = cv::Mat::zeros(1, 5, CV_64F);
    camera.translation_camera_to_gimbal = cv::Mat::zeros(1, 3, CV_64F);
    assert(!camera.valid());
    camera.translation_camera_to_gimbal = cv::Mat::zeros(3, 1, CV_64F);
    assert(camera.valid());

    // 无目标输出必须满足协议规定的安全不变量。
    Translator message{};
    message.message.yaw = 0.25F;
    message.message.pitch = -0.1F;
    setSafeOutput(message);
    assert(message.message.head == kFrameHead);
    assert(message.message.tail == kFrameTail);
    assert(message.message.target_yaw == message.message.yaw);
    assert(message.message.target_pitch == message.message.pitch);
    assert(message.message.fire_allowance == 0U);
    assert(message.message.crc == 0U);

    Translator invalid_input{};
    invalid_input.message.yaw = std::numeric_limits<float>::quiet_NaN();
    invalid_input.message.pitch = std::numeric_limits<float>::infinity();
    invalid_input.message.roll = -std::numeric_limits<float>::infinity();
    invalid_input.message.bias = std::numeric_limits<float>::quiet_NaN();
    setSafeOutput(invalid_input);
    assert(std::isfinite(invalid_input.message.yaw));
    assert(std::isfinite(invalid_input.message.pitch));
    assert(std::isfinite(invalid_input.message.roll));
    assert(std::isfinite(invalid_input.message.bias));
    assert(std::isfinite(invalid_input.message.target_yaw));
    assert(std::isfinite(invalid_input.message.target_pitch));
    assert(invalid_input.message.fire_allowance == 0U);

    // 有效预测应将国际单位制数值映射到旧协议单位和跟踪状态。
    PredictionResult stable;
    stable.state = TrackingState::Stable;
    stable.target_id = 3U;
    stable.center_x_m = 1.25F;
    stable.center_y_m = -0.5F;
    stable.distance_m = 4.5F;
    stable.target_yaw = 0.4F;
    stable.target_pitch = -0.2F;
    stable.fire_allowed = true;
    assert(applyPrediction(message, stable));
    assert(message.message.armor_flag == 3U);
    assert(std::abs(message.message.coo_x - 1250.0F) < 1e-4F);
    assert(std::abs(message.message.coo_y + 500.0F) < 1e-4F);
    assert(message.message.fire_allowance == 1U);
    assert(message.message.crc == 1U);

    stable.state = TrackingState::Unstable;
    assert(applyPrediction(message, stable));
    assert(message.message.fire_allowance == 0U);
    assert(message.message.crc == 2U);

    stable.target_yaw = std::numeric_limits<float>::quiet_NaN();
    assert(!applyPrediction(message, stable));
    assert(message.message.fire_allowance == 0U);
    assert(message.message.crc == 0U);

    // 状态决定敌方颜色；控制器的时序偏置应传递到 EKF 输入。
    assert(isAutoAimStatus(0U));
    assert(isAutoAimStatus(5U));
    assert(!isAutoAimStatus(1U));
    assert(enemyColorFromStatus(0U) == TeamColor::Blue);
    assert(enemyColorFromStatus(5U) == TeamColor::Red);

    MessData_AutoAim controller_message{};
    controller_message.yaw = 0.1F;
    controller_message.pitch = 0.2F;
    controller_message.roll = 0.3F;
    controller_message.bias = 0.04F;
    const GimbalState gimbal = gimbalStateFromMessage(controller_message);
    assert(gimbal.yaw == controller_message.yaw);
    assert(gimbal.pitch == controller_message.pitch);
    assert(gimbal.roll == controller_message.roll);
    assert(gimbal.prediction_bias_s == controller_message.bias);
    return 0;
}
