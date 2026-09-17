#include "AppSupport.hpp"

#include <cmath>

namespace {

float finiteOrZero(float value) noexcept {
    return std::isfinite(value) ? value : 0.0F;
}

bool finitePrediction(const PredictionResult& prediction) {
    // 同时检查国际单位制数值，以及米转换为毫米后的 float 通信表示。
    const float wire_center_x_mm = prediction.center_x_m * 1000.0F;
    const float wire_center_y_mm = prediction.center_y_m * 1000.0F;
    return prediction.target_id != 0U &&
           (prediction.state == TrackingState::Stable ||
            prediction.state == TrackingState::Unstable) &&
           std::isfinite(prediction.center_x_m) &&
           std::isfinite(prediction.center_y_m) &&
           std::isfinite(wire_center_x_mm) && std::isfinite(wire_center_y_mm) &&
           std::isfinite(prediction.center_velocity_x_m_s) &&
           std::isfinite(prediction.center_velocity_y_m_s) &&
           std::isfinite(prediction.body_yaw) &&
           std::isfinite(prediction.body_yaw_velocity) &&
           std::isfinite(prediction.radius_1_m) &&
           std::isfinite(prediction.radius_2_m) && prediction.radius_1_m >= 0.0F &&
           prediction.radius_2_m >= 0.0F &&
           std::isfinite(prediction.distance_m) && std::isfinite(prediction.target_yaw) &&
           std::isfinite(prediction.target_pitch) &&
           std::isfinite(prediction.target_yaw_velocity) &&
           std::isfinite(prediction.target_pitch_velocity) && prediction.distance_m >= 0.0F;
}

}  // 匿名命名空间

bool isAutoAimStatus(std::uint8_t status) noexcept {
    return status == 0U || status == 5U;
}

TeamColor enemyColorFromStatus(std::uint8_t status) noexcept {
    const bool own_team_is_blue = status == 5U;
    return own_team_is_blue ? TeamColor::Red : TeamColor::Blue;
}

GimbalState gimbalStateFromMessage(const MessData_AutoAim& message) noexcept {
    return {message.yaw, message.pitch, message.roll, message.bias};
}

void setSafeOutput(Translator& translator) noexcept {
    // 安全不变量：无目标、无运动前馈，并且绝不允许开火。
    // 控制器提供的姿态/偏置字段在有限时保留，便于诊断。
    auto& message = translator.message;
    message.head = kFrameHead;
    message.yaw = finiteOrZero(message.yaw);
    message.pitch = finiteOrZero(message.pitch);
    message.roll = finiteOrZero(message.roll);
    message.bias = finiteOrZero(message.bias);
    message.armor_flag = 0U;
    message.latency = 0.0F;
    message.distance = 0.0F;
    message.pitch_offset = 0.0F;
    message.coo_x = 0.0F;
    message.coo_y = 0.0F;
    message.wheel_w = 0.0F;
    message.fire_allowance = 0U;
    message.target_yaw = message.yaw;
    message.target_pitch = message.pitch;
    message.yaw_vel = 0.0F;
    message.pitch_vel = 0.0F;
    message.crc = static_cast<std::uint16_t>(TrackingState::NoTarget);
    message.tail = kFrameTail;
}

bool applyPrediction(Translator& translator, const PredictionResult& prediction) noexcept {
    // 学生结果只要不完整或包含非有限值，就降级为同样的安全帧。
    if (prediction.state == TrackingState::NoTarget || !finitePrediction(prediction)) {
        setSafeOutput(translator);
        return false;
    }

    auto& message = translator.message;
    message.head = kFrameHead;
    message.armor_flag = prediction.target_id;
    message.distance = prediction.distance_m;
    message.pitch_offset = 0.0F;
    // 旧协议的 coo_x/coo_y 字段单位为毫米；内部算法统一使用米。
    message.coo_x = prediction.center_x_m * 1000.0F;
    message.coo_y = prediction.center_y_m * 1000.0F;
    message.wheel_w = 0.0F;
    message.target_yaw = prediction.target_yaw;
    message.target_pitch = prediction.target_pitch;
    message.yaw_vel = prediction.target_yaw_velocity;
    message.pitch_vel = prediction.target_pitch_velocity;
    message.crc = static_cast<std::uint16_t>(prediction.state);
    message.fire_allowance = prediction.state == TrackingState::Stable &&
                                     prediction.fire_allowed
                                 ? 1U
                                 : 0U;
    message.tail = kFrameTail;
    return true;
}
