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

    // 设置初始状态。
    void setState(const Eigen::VectorXd & x0);

    // 根据过程模型计算预测状态。
    Eigen::MatrixXd predict();

    // 根据观测量更新估计状态。
    Eigen::MatrixXd update(const Eigen::VectorXd & z);

    // 使用自定义观测模型（z、H、R）更新。
    Eigen::MatrixXd update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R);

    // 将后验状态作为下一次更新的先验状态。
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
    // 非线性过程函数。
    VecVecFunc f;
    // 非线性观测函数。
    VecVecFunc h;
    // 过程函数 f() 的雅可比矩阵。
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    // 观测函数 h() 的雅可比矩阵。
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    // 过程噪声协方差矩阵。
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    // 观测噪声协方差矩阵。
    VecMatFunc update_R;
    Eigen::MatrixXd R;
    // 观测残差归一化函数。
    VecVecFunc nomolize_residual;

    // 先验误差协方差矩阵。
    Eigen::MatrixXd P_pri;
    // 后验误差协方差矩阵。
    Eigen::MatrixXd P_post;

    // 卡尔曼增益。
    Eigen::MatrixXd K;

    // 状态维度。
    int n;

    // N 维单位矩阵。
    Eigen::MatrixXd I;

    // 先验状态。
    Eigen::VectorXd x_pri;
    // 后验状态。
    Eigen::VectorXd x_post;
};
#endif // _KALMANFILTER_HPP
