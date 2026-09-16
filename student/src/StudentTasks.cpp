#include "StudentTasks.hpp"

bool get_pic(cv::Mat& pic) {
    // TODO(student): initialize/open/start the camera once, then acquire and convert
    // one frame to BGR here. Release SDK buffers on every success and failure path.
    pic.release();
    return false;
}

std::vector<ArmorDetection> armor_detect(const cv::Mat&, TeamColor) {
    // TODO(student): implement traditional light-bar matching plus classification,
    // or an offline neural detector. Reject malformed/out-of-image corner sets.
    return {};
}

std::vector<ArmorPose> armor_solve(const std::vector<ArmorDetection>&,
                                  const CameraParameters&, const GimbalState&) {
    // TODO(student): build size-specific object points (mm -> m), run solvePnP,
    // reject bad depth/reprojection error, then apply the calibrated rigid transform.
    return {};
}

PredictionResult ekf_predict(const std::vector<ArmorPose>&, const GimbalState&,
                             double) {
    // TODO(student): handle association, initialization, angle wrapping, outliers,
    // armor switching, short loss and timeout reset before future-state prediction.
    return {};
}
