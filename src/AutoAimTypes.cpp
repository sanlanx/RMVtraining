#include "AutoAimTypes.hpp"

#include <cmath>
#include <sstream>

namespace {

bool isMatrix(const cv::Mat& value, int rows, int cols) {
    return !value.empty() && value.type() == CV_64F && value.rows == rows &&
           value.cols == cols;
}

bool isDistortionVector(const cv::Mat& value) {
    if (value.empty() || value.type() != CV_64F ||
        (value.rows != 1 && value.cols != 1)) {
        return false;
    }
    // Coefficient counts accepted by OpenCV's pinhole distortion model.
    const std::size_t coefficients = value.total();
    return coefficients == 4U || coefficients == 5U || coefficients == 8U ||
           coefficients == 12U || coefficients == 14U;
}

}  // namespace

bool CameraParameters::valid() const {
    if (!calibrated || image_width <= 0 || image_height <= 0 ||
        !isMatrix(camera_matrix, 3, 3) ||
        !isDistortionVector(distortion_coefficients) ||
        !isMatrix(rotation_camera_to_gimbal, 3, 3) ||
        !isMatrix(translation_camera_to_gimbal, 3, 1) ||
        !std::isfinite(mean_reprojection_error_px) || mean_reprojection_error_px < 0.0 ||
        !cv::checkRange(camera_matrix) || !cv::checkRange(distortion_coefficients) ||
        !cv::checkRange(rotation_camera_to_gimbal) ||
        !cv::checkRange(translation_camera_to_gimbal)) {
        return false;
    }

    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double rotation_determinant = cv::determinant(rotation_camera_to_gimbal);
    // A rigid transform requires an orthonormal rotation with determinant +1.
    const cv::Mat orthogonality = rotation_camera_to_gimbal.t() *
                                  rotation_camera_to_gimbal - cv::Mat::eye(3, 3, CV_64F);
    return fx > 0.0 && fy > 0.0 && std::isfinite(rotation_determinant) &&
           std::abs(rotation_determinant - 1.0) < 1e-2 &&
           cv::norm(orthogonality, cv::NORM_INF) < 1e-2;
}

bool loadCameraParameters(const std::string& path, CameraParameters& parameters,
                          std::string& error) {
    cv::FileStorage storage(path, cv::FileStorage::READ);
    if (!storage.isOpened()) {
        error = "cannot open camera parameter file: " + path;
        return false;
    }

    int calibrated = 0;
    storage["calibrated"] >> calibrated;
    parameters.calibrated = calibrated == 1;
    storage["image_width"] >> parameters.image_width;
    storage["image_height"] >> parameters.image_height;
    storage["camera_matrix"] >> parameters.camera_matrix;
    storage["distortion_coefficients"] >> parameters.distortion_coefficients;
    storage["rotation_camera_to_gimbal"] >> parameters.rotation_camera_to_gimbal;
    storage["translation_camera_to_gimbal"] >> parameters.translation_camera_to_gimbal;
    storage["mean_reprojection_error_px"] >> parameters.mean_reprojection_error_px;

    // Normalize YAML numeric types before validation and later OpenCV calculations.
    if (!parameters.camera_matrix.empty()) {
        parameters.camera_matrix.convertTo(parameters.camera_matrix, CV_64F);
    }
    if (!parameters.distortion_coefficients.empty()) {
        parameters.distortion_coefficients.convertTo(parameters.distortion_coefficients, CV_64F);
    }
    if (!parameters.rotation_camera_to_gimbal.empty()) {
        parameters.rotation_camera_to_gimbal.convertTo(parameters.rotation_camera_to_gimbal, CV_64F);
    }
    if (!parameters.translation_camera_to_gimbal.empty()) {
        parameters.translation_camera_to_gimbal.convertTo(parameters.translation_camera_to_gimbal,
                                                           CV_64F);
    }

    if (!parameters.valid()) {
        std::ostringstream message;
        message << "camera parameters in " << path
                << " are incomplete; fill the calibration template before running";
        error = message.str();
        return false;
    }
    return true;
}
