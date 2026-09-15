#include "BulletModel.hpp"
#include <cmath>

Bullet_Model::Bullet_Model(float dist, float height)
    : dist_x(dist), height_z(height), velo(24.f),
      iter_num(0), beta(0), alpha_inter(0), err_max(0.0001f),
      z_iter(height_z), z_iter_last(height_z), z_diff(0), flight_time(0)
{
}

float Bullet_Model::bullet_iteration_cal()
{
    constexpr int kMaxIterations = 100;
    iter_num = 0;

    if (dist_x < 0.001f)
        dist_x = 0.001f; // Prevent zero distance (1mm minimum)
    beta = atan2f(height_z, dist_x);
    z_iter_last = height_z;
    do
    {
        float cos_beta = cosf(beta);
        if (fabsf(cos_beta) < 1e-6)
            cos_beta = 1e-6; // Prevent division by zero

        float temp = (dist_x * Kf) / (velo * cos_beta);
        if (temp >= 0.999f)
            temp = 0.999f; // Prevent log of non-positive

        alpha_inter = (float)(1 - temp);
        z_iter = (float)dist_x * tanf(beta) + (dist_x * gravity) / (Kf * velo * cos_beta) + gravity * logf(alpha_inter) / powf(Kf, 2);
        iter_num++; // 迭代次数加一
        /// 下次迭代用的数据
        z_diff = z_iter - height_z;
        z_iter = z_iter_last - z_diff;
        beta = atan2f(z_iter_last - z_diff, dist_x);
        z_iter_last = z_iter; // 这里将z_iter赋值成迭代后的beta*dist_x
    } while (fabsf(z_diff) >= err_max && iter_num < kMaxIterations); // 默认理论迭代精度0.1mm
    float cos_beta = cosf(beta);
    if (fabsf(cos_beta) < 1e-6)
        cos_beta = 1e-6f;
    float temp = (dist_x * Kf) / (velo * cos_beta);
    if (temp >= 0.999f)
        temp = 0.999f;
    flight_time = (-1 / Kf) * logf(1 - temp);
    return flight_time;
}
