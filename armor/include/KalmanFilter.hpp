#ifndef _KALMANFILTER_HPP
#define _KALMANFILTER_HPP

#include <Eigen/Dense>
#include <functional>

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter() = default;

    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    explicit ExtendedKalmanFilter(
        const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
        const VoidMatFunc & u_q, const VecMatFunc & u_r, const VecVecFunc & nomolize_residual,
        const Eigen::MatrixXd & P0, const Eigen::VectorXd & x0);

    // Set the initial state
    void setState(const Eigen::VectorXd & x0);

    // Compute a predicted state
    Eigen::MatrixXd predict();

    // Update the estimated state based on measurement
    Eigen::MatrixXd update(const Eigen::VectorXd & z);

    // Update with custom measurement model (z, H, R)
    Eigen::MatrixXd update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R);

    // Use posterior as prior (for sequential updates)
    void usePostAsPri() {
        x_pri = x_post;
        P_pri = P_post;
    }

    Eigen::VectorXd& get_X(){
        return x_post;
    }
    const Eigen::VectorXd& get_nochange_X() const{
        return x_post;
    }
    
    // 获取可修改的先验状态引用（用于神经网络修正）
    Eigen::VectorXd& get_X_pri(){
        return x_pri;
    }
    const Eigen::VectorXd& get_nochange_X_pri() const{
        return x_pri;
    }

    // 获取后验协方差矩阵（用于双EKF竞争评估）
    const Eigen::MatrixXd& get_P_post() const {
        return P_post;
    }
    // 获取先验协方差矩阵
    const Eigen::MatrixXd& get_P_pri() const {
        return P_pri;
    }

    private:
    // Process nonlinear vector function
    VecVecFunc f;
    // Observation nonlinear vector function
    VecVecFunc h;
    // Jacobian of f()
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    // Jacobian of h()
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    // Process noise covariance matrix
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    // Measurement noise covariance matrix
    VecMatFunc update_R;
    Eigen::MatrixXd R;
    // Nomalization residual for measurement
    VecVecFunc nomolize_residual;

    // Priori error estimate covariance matrix
    Eigen::MatrixXd P_pri;
    // Posteriori error estimate covariance matrix
    Eigen::MatrixXd P_post;

    // Kalman gain
    Eigen::MatrixXd K;

    // System dimensions
    int n;

    // N-size identity
    Eigen::MatrixXd I;

    // Priori state
    Eigen::VectorXd x_pri;
    // Posteriori state
    Eigen::VectorXd x_post;
};
#endif //_KALMANFILTER_HPP