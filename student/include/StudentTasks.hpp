#pragma once

#include "AutoAimTypes.hpp"

#include <vector>

#include <opencv2/core.hpp>

// 任务 1：管理相机 SDK，返回一张非空的 CV_8UC3 BGR 图像。
// 超时或断开时返回 false；不要在此函数中终止进程。
bool get_pic(cv::Mat& pic);

// 任务 2：检测敌方装甲板并识别其编号。每个结果必须包含四个按顺时针排列的
// 图像角点、装甲板尺寸、目标编号和置信度。
std::vector<ArmorDetection> armor_detect(const cv::Mat& pic, TeamColor enemy_color);

// 任务 3：使用已标定的相机内参进行位姿解算，再将观测从相机坐标系变换到云台坐标系。
// 输出位置单位为米。
std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>& detections,
                                  const CameraParameters& camera,
                                  const GimbalState& gimbal);

// 任务 4：完成观测关联，更新旋转目标 EKF，并预测云台指令。
// timestamp_seconds 单调递增；prediction_bias_s 来自控制器。
PredictionResult ekf_predict(const std::vector<ArmorPose>& observations,
                             const GimbalState& gimbal, double timestamp_seconds);
