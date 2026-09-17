#pragma once

#include "AutoAimTypes.hpp"
#include "Communication.hpp"

#include <cstdint>

// 本作业只处理状态 0（我方红）和 5（我方蓝）。
bool isAutoAimStatus(std::uint8_t status) noexcept;
// status 编码我方颜色；装甲板识别需要使用相反的敌方颜色。
TeamColor enemyColorFromStatus(std::uint8_t status) noexcept;
// 提取由控制器提供的姿态和预测时序字段。
GimbalState gimbalStateFromMessage(const MessData_AutoAim& message) noexcept;

// 生成确定性的无目标帧，并清空所有与开火相关的字段。
void setSafeOutput(Translator& translator) noexcept;
// 校验 EKF 结果，并将国际单位制单位映射到保持不变的通信协议。
// 特别地，center_x/y 会从米转换为毫米。
bool applyPrediction(Translator& translator, const PredictionResult& prediction) noexcept;
