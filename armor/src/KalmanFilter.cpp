#include "KalmanFilter.hpp"
#include "iostream"

ExtendedKalmanFilter::ExtendedKalmanFilter(
  const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
  const VoidMatFunc & u_q, const VecMatFunc & u_r, const VecVecFunc & nomolize_residual,
  const Eigen::MatrixXd & P0, const Eigen::VectorXd & x0)
: f(f),
  h(h),
  jacobian_f(j_f),
  jacobian_h(j_h),
  update_Q(u_q),
  update_R(u_r),
  nomolize_residual(nomolize_residual),
  P_pri(P0),
  P_post(P0),
  n(P0.rows()),
  I(Eigen::MatrixXd::Identity(n, n)),
  x_pri(x0),
  x_post(x0)
{
}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
  F = jacobian_f(x_post), Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
{
  H = jacobian_h(x_pri), R = update_R(z);
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * nomolize_residual(z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, const Eigen::MatrixXd & H_custom, const Eigen::MatrixXd & R_custom)
{
  H = H_custom;
  R = R_custom;
  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - H * x_pri); // Assume linear residual
  P_post = (I - K * H) * P_pri;

  return x_post;
}
