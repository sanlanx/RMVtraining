#include "StudentTasks.hpp"

bool get_pic(cv::Mat& pic) {
    // TODO(student)：在这里完成相机的一次性初始化/打开/启动，随后获取一帧并转换为 BGR。
    // 无论成功还是失败，都要释放 SDK 缓冲区。
    pic.release();
    return false;
}

std::vector<ArmorDetection> armor_detect(const cv::Mat&, TeamColor) {
    // TODO(student)：实现传统灯条匹配与数字分类，或使用离线神经网络检测器。
    // 拒绝格式错误或超出图像范围的角点集合。
    return {};
}

std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>&,
                                  const CameraParameters&, const GimbalState&) {
    // TODO(student)：按装甲板尺寸建立物点（毫米转换为米），调用 solvePnP，
    // 拒绝深度或重投影误差异常的结果，再应用已标定的刚体变换。
    return {};
}

PredictionResult ekf_predict(const std::vector<ArmorPose>&, const GimbalState&,
                             double) {
    // TODO(student)：在未来状态预测前处理观测关联、初始化、角度归一化、离群点、
    // 装甲板切换、短时丢失以及超时重置。
    return {};
}
