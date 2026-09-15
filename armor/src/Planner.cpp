#include "Planner.hpp"
#include <cmath>
#include <iostream>
#include <cstdlib>
#include <chrono>
#include <algorithm>
#include <future>
#include "BulletModel.hpp"
#include "WebLogger.hpp"

Planner::Planner(const PlannerConfig& config)
    : Planner(config, Eigen::VectorXd::Zero(13), 0)
{}

Planner::Planner(const PlannerConfig& config, const Eigen::VectorXd& x, int num)
    : config_(config)
    , EKF_statue(x)
    , number(num)
{
    yaw_history_.clear();
}

Planner::~Planner()
{}

void Planner::setConfig(const PlannerConfig& config)
{
    config_ = config;
}

PlannerConfig Planner::getConfig() const
{
    return config_;
}

Plan Planner::plan(double bullet_speed, Translator &ts , float dt)
{
    Plan plan{};
    plan.control = false;
    
    // Prevent division by zero and NaNs from dt
    if (dt <= 1e-6) dt = config_.dt;

    // 0. 子弹速度保护
    if (bullet_speed < 10.0 || bullet_speed > 25.0) {
        bullet_speed = 23.0;
    }
    
    // 1. 使用新的方法计算目标yaw和pitch（与驱动文件一致）
    Eigen::Vector2d target_yaw_pitch = getTargetYawPitch(bullet_speed, ts);
    // cout << "target_yaw: " << target_yaw_pitch(0) << "target_pitch: " << target_yaw_pitch(1) << endl;
    
    // 5. 从HALF_HORIZON获取MPC规划结果
    plan.control = true;
    
    // 目标角度（直接使用新方法计算的目标）
    plan.target_yaw = target_yaw_pitch(0);
    plan.target_pitch = target_yaw_pitch(1);
    plan.yaw_vel = ts.message.yaw_vel;
    // printf("---------------------\n");
    return plan;
}

/**
 * @brief 获取目标yaw和pitch（与outpost_predict_driver和vision_predict_driver保持一致）
 * @param bullet_speed 子弹速度 m/s
 * @param ts Translator对象，用于获取当前云台角度
 * @return Eigen::Vector2d 包含目标yaw和pitch（弧度）
 */
Eigen::Vector2d Planner::getTargetYawPitch(double bullet_speed, Translator &ts)
{
    Eigen::Vector2d result(0, 0);
    
    // 子弹速度保护
    if (bullet_speed < 10.0 || bullet_speed > 25.0) {
        bullet_speed = 23.0;
    }
    
    // 设置偏置（参考驱动文件：vision 0.065, outpost 0.61）
    double bias = ts.message.bias;  // 偏置，单位为秒
    int n = (number == 5) ? 3 : 4;
    int cnt;

    Bullet_Model bullet_model(0, 0);
    for (int index = 0; index < n; index ++) {
        t_last = t = 0;
        cnt = 0;
        while (true) {
            // 优化：只计算当前关注的装甲板的位置
            trackLine(t + bias, EKF_statue, ts, index);
            transform(index, ts.message.pitch, ts.message.yaw);
            double horizontal_distance = sqrt(pow(aim_armor_coo(0), 2) + pow(aim_armor_coo(1), 2)) / 1000.0;
            double height = aim_armor_coo(2) / 1000.0;
            bullet_model.dist_x = horizontal_distance;
            bullet_model.height_z = height;
            bullet_model.velo = bullet_speed; // 设置子弹速度
            bullet_model.iter_num = 0;
            t = bullet_model.bullet_iteration_cal();
            cnt++;
            if (std::fabs(t - t_last) < 0.001 || cnt >= 10) {
                time[index] = t + bias;
                // cout << "fly_time: " << t + bias << endl;
                break;
            }
            t_last = t;
        }
    }
    
    const int previous_target_armor_index = target_armor_index;
    target_armor_index = SelectShoot(EKF_statue, ts);
    const bool armor_switched = previous_target_armor_index >= 0 &&
                                previous_target_armor_index != target_armor_index;
    trackLine(time[target_armor_index], EKF_statue, ts);
    transform(target_armor_index, ts.message.pitch, ts.message.yaw);
    double target_yaw_vel = 0.0;
    if (number != 5) {
        target_yaw_vel = calculateTargetYawVelocity(target_armor_index, time[target_armor_index], EKF_statue, ts);
    }
    double horizontal_distance_last = sqrt(pow(aim_armor_coo(0), 2) + pow(aim_armor_coo(1), 2)) / 1000.0;
    double height_last = aim_armor_coo(2) / 1000.0;
    Bullet_Model final_bullet_model(horizontal_distance_last, height_last);
    final_bullet_model.velo = bullet_speed;
    t = final_bullet_model.bullet_iteration_cal();
    result(1) = final_bullet_model.beta;

    result(0) = atan2(aim_armor_coo(1), aim_armor_coo(0));

    // cout << "aim_armor_coo(1): " << aim_armor_coo(1) << "  aim_armor_coo(0): " << aim_armor_coo(0) << endl;

    result(0) = limit_rad(result(0));

    if (armor_switched) {
        armor_switching_ = true;
    }

    constexpr double SWITCH_CAPTURE_YAW_ERROR = 0.04;
    constexpr double SWITCH_YAW_VELOCITY_GAIN = 8.0;
    constexpr double SWITCH_YAW_VELOCITY_MAX = 1.5;
    const double yaw_error = limit_rad(result(0) - ts.message.yaw);
    if (armor_switching_) {
        if (std::fabs(yaw_error) < SWITCH_CAPTURE_YAW_ERROR) {
            armor_switching_ = false;
        } else {
            target_yaw_vel = std::clamp(
                SWITCH_YAW_VELOCITY_GAIN * yaw_error,
                -SWITCH_YAW_VELOCITY_MAX,
                SWITCH_YAW_VELOCITY_MAX);
        }
    }
    
    ts.message.target_yaw = result(0);
    ts.message.target_pitch = result(1);
    ts.message.yaw_vel = static_cast<float>(target_yaw_vel);
    WEB_LOG("target_yaw", result(0));
    WEB_LOG("target_pitch", result(1));
    WEB_LOG("target_yaw_vel", target_yaw_vel);
    // cout << "result_yaw: " << result(0) << endl;

    return result;
}

void Planner::trackLine(float time, Eigen::VectorXd& x, Translator &ts, int target_idx)
{
    // float wheel_r;        // 轮腿半径，单位为毫米
    // float wheel_angle;    // 轮腿相位，单位为弧度
    // float wheel_w;        // 轮腿自转角速度，单位为弧度每秒，逆时针为正
    t = time;
    float latency = ts.message.latency / 1000.0f;
    float total_t = t + latency;
    // cout << "wwheeled_w: " << ts.message.wheel_w << endl;

    // 计算中心位置
    // ts.message.wheel_w = 0;
    // if (ts.message.wheel_w == 0) {
    center.x = x(0) + x(1) * total_t;
    center.y = x(2) + x(3) * total_t;
    center.z = 0;
    WEB_LOG("xv", x(1));
    WEB_LOG("yv", x(3));
    WEB_LOG("vyaw", x(10));
    // }
    // else {
    //     float v_line = ts.message.wheel_w * ts.message.wheel_r * 0.2; // 轮腿线速度 mm/s
    //     float v_x = x(1) - v_line * std::cos(ts.message.wheel_angle + M_PI / 2);
    //     float v_y = x(3) - v_line * std::sin(ts.message.wheel_angle + M_PI / 2);
    //     float v_x1 = v_line * std::cos(ts.message.wheel_angle + M_PI / 2 + ts.message.wheel_w * (ts.message.bias + latency));
    //     float v_y1 = v_line * std::sin(ts.message.wheel_angle + M_PI / 2 + ts.message.wheel_w * (ts.message.bias + latency));
    //     // center.x = x(0) + v_x * total_t + ts.message.wheel_r * std::cos(ts.message.wheel_angle + ts.message.wheel_w * (ts.message.bias + latency)) - ts.message.wheel_r * std::cos(ts.message.wheel_angle)
    //     //         + v_x1 * (time - ts.message.bias);
    //     center.x = x(0) + v_x * total_t - ts.message.wheel_r * std::cos(ts.message.wheel_angle);
    //     // center.y = x(2) + v_y * total_t + ts.message.wheel_r * std::sin(ts.message.wheel_angle + ts.message.wheel_w * (ts.message.bias + latency)) - ts.message.wheel_r * std::sin(ts.message.wheel_angle)
    //     //         + v_y1 * (time - ts.message.bias);
    //     center.y = x(2) + v_y * total_t - ts.message.wheel_r * std::sin(ts.message.wheel_angle);
    //     center.z = 0;
    //     ts.message.yaw_vel = atan2(x(3), x(1));     
    //     ts.message.pitch_vel = sqrtf(powf(x(1), 2) + powf(x(3), 2)) / ts.message.wheel_w;
    //     // cout << "wheeled_angle: " << ts.message.wheel_angle <<endl;
    //     // cout << "x(3) / x(1): " << atan2(x(3), x(1)) << endl;
    //     // cout << "...........: " << atan2(std::sin(ts.message.wheel_angle), std::cos(ts.message.wheel_angle)) << endl;
    // }
    auto angle = atan2(center.y, center.x); 
    if (number == 5) {
        // 前哨站: 三个装甲板高度由EKF独立维护
        double outpost_r = config_.outpost_r;
        
        if (target_idx == -1 || target_idx == 0) {
            armors_xyz[0] = cv::Point3f(center.x - outpost_r * std::cos(x(9) + x(10) * (total_t)),
                                    center.y - outpost_r * std::sin(x(9) + x(10) * (total_t)),
                                    x(4));
        }
        if (target_idx == -1 || target_idx == 1) {
            armors_xyz[1] = cv::Point3f(center.x - outpost_r * std::cos(x(9) + x(10) * (total_t) + M_PI * 2 / 3),
                                    center.y - outpost_r * std::sin(x(9) + x(10) * (total_t) + M_PI * 2 / 3),
                                    x(5));
        }
        if (target_idx == -1 || target_idx == 2) {
            armors_xyz[2] = cv::Point3f(center.x - outpost_r * std::cos(x(9) + x(10) * (total_t) + M_PI * 4 / 3),
                                    center.y - outpost_r * std::sin(x(9) + x(10) * (total_t) + M_PI * 4 / 3),
                                    x(6));
        }
    }
    else {
        double z0 = x(4);
        double z1 = x(5);
        double z2 = (x.size() > 11) ? x(11) : x(4);
        double z3 = (x.size() > 12) ? x(12) : x(5);

        if (target_idx == -1 || target_idx == 0) {
            armors_xyz[0] = cv::Point3f(center.x - x(7) * std::cos(x(9) + x(10) * (total_t)),
                                    center.y - x(7) * std::sin(x(9) + x(10) * (total_t)),
                                    z0);
        }
        if (target_idx == -1 || target_idx == 1) {
            armors_xyz[1] = cv::Point3f(center.x - x(8) * std::cos(x(9) + x(10) * (total_t) + M_PI / 2),
                                    center.y - x(8) * std::sin(x(9) + x(10) * (total_t) + M_PI / 2),
                                    z1);
        }
        if (target_idx == -1 || target_idx == 2) {
            armors_xyz[2] = cv::Point3f(center.x - x(7) * std::cos(x(9) + x(10) * (total_t) + M_PI),
                                    center.y - x(7) * std::sin(x(9) + x(10) * (total_t) + M_PI),
                                    z2);
        }
        if (target_idx == -1 || target_idx == 3) {
            armors_xyz[3] = cv::Point3f(center.x - x(8) * std::cos(x(9) + x(10) * (total_t) + M_PI * 3 / 2),
                                    center.y - x(8) * std::sin(x(9) + x(10) * (total_t) + M_PI * 3 / 2),
                                    z3);
        }
    }
}

void Planner::transform(int armor_num, float pitch_angle, float yaw_angle)
{
    // Eigen::Matrix3f pitch_tran = Eigen::AngleAxisf(-pitch_angle, Eigen::Vector3f::UnitY()).toRotationMatrix();
    // Eigen::Matrix3f yaw_tran = Eigen::AngleAxisf(-yaw_angle , Eigen::Vector3f::UnitZ()).toRotationMatrix();
    // Eigen::Matrix3f tran = pitch_tran * yaw_tran;

    aim_armor_coo << armors_xyz[armor_num].x, armors_xyz[armor_num].y, armors_xyz[armor_num].z;
    // Eigen::Vector3f result = tran * (armor_eigen - translation_vector);
    // aim_armor_coo = aim_armor_coo + tran * translation_vector;
    Eigen::Vector3f result = {- translation_vector(0) * std::cos(pitch_angle) * std::cos(yaw_angle),
                            - translation_vector(0) * std::cos(pitch_angle) * std::sin(yaw_angle),
                            - translation_vector(0) * std::sin(pitch_angle)};
    aim_armor_coo = aim_armor_coo + result;
}

double Planner::calculateTargetYawVelocity(int armor_num, float predict_time, const Eigen::VectorXd &x, const Translator &ts) const
{
    if (number == 5) {
        return 0.0;
    }

    if (armor_num < 0 || armor_num >= ((number == 5) ? 3 : 4) || x.size() <= 10) {
        return 0.0;
    }

    const double latency = ts.message.latency / 1000.0;
    const double total_t = static_cast<double>(predict_time) + latency;
    const double armor_offset = (number == 5) ? armor_num * M_PI * 2.0 / 3.0
                                              : armor_num * M_PI / 2.0;
    const double armor_yaw = x(9) + x(10) * total_t + armor_offset;
    const double radius = (number == 5) ? config_.outpost_r
                                        : ((armor_num % 2 == 0) ? x(7) : x(8));

    const double armor_vx = x(1) + radius * std::sin(armor_yaw) * x(10);
    const double armor_vy = x(3) - radius * std::cos(armor_yaw) * x(10);

    const double barrel_offset_x = -translation_vector(0) * std::cos(ts.message.pitch) * std::cos(ts.message.yaw);
    const double barrel_offset_y = -translation_vector(0) * std::cos(ts.message.pitch) * std::sin(ts.message.yaw);
    const double aim_x = armors_xyz[armor_num].x + barrel_offset_x;
    const double aim_y = armors_xyz[armor_num].y + barrel_offset_y;
    const double dist2 = aim_x * aim_x + aim_y * aim_y;

    if (dist2 < 1e-6) {
        return 0.0;
    }

    return (aim_x * armor_vy - aim_y * armor_vx) / dist2;
}

void Planner::getTrackArmorsXYZ(std::vector<cv::Point3f> &points, std::vector<double> &yaws)
{
    points.clear();
    yaws.clear();
    int n = (number == 5) ? 3 : 4;
    
    for (int i = 0; i < n; i++) {
        points.push_back(armors_xyz[i]);
        // 计算装甲板yaw
        double armor_yaw = EKF_statue(9) + EKF_statue(10) * time[i];
        if (number == 5) {
            armor_yaw += i * M_PI * 2 / 3;
        } else {
            armor_yaw += i * M_PI / 2;
        }
        yaws.push_back(armor_yaw);
    }
}

int Planner::SelectShoot(Eigen::VectorXd& x,  Translator &ts)
{
    // const double ANGLE_TOLERANCE = 0.05;
    // 先把标记设为false，只有在计算并打印角度时设为true
    // angles_valid_ = false;
    double dt;  //当前观测到子弹命中的时间
    float latency = ts.message.latency / 1000;
    averate_r = x(7) * 0.6 + x(8) * 0.4;
    if (number == 5) {
        double angle[3];
        dt = time[0] + latency;
        angle[0] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt);
        dt = time[1] + latency;
        angle[1] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt) + M_PI * 2 / 3;
        dt = time[2] + latency;
        angle[2] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt) + M_PI * 4 / 3;
        for (int i = 0; i < 3; i ++) {
            angle[i] += 0.1 * x(10);
            angle[i] = atan2(sin(angle[i]), cos(angle[i]));
        }
        // --- NEW: 使用飞行时间(距离)过滤 ---
        // 1. 创建索引列表并按飞行时间排序(从短到长)
        std::vector<int> candidates = {0, 1, 2};
        std::sort(candidates.begin(), candidates.end(), [this](int a, int b) {
            return this->time[a] < this->time[b];
        });
        // 2. 只保留距离最近的两个装甲板作为候选
        candidates.resize(2); 

        // 3. 在候选装甲板中寻找角度最小的
        // 注：3板情况下所有装甲板半径相同(x(7))，无需考虑半径优先
        int index = candidates[0];
        double abs_min = fabs(angle[index]);
        for (int idx : candidates) {
            if (fabs(angle[idx]) < abs_min) {
                abs_min = fabs(angle[idx]);
                index = idx;
            }
        }
        
        // 4. 计算角度限制并设置flag
        dt = time[index] + latency;
        double theta_relative = atan2(x(2) + x(3) * dt, x(0) + x(1) * dt);
        double center_dist = sqrt(pow(center.x, 2) + pow(center.y, 2)) / 1000.0;  // 中心距离(m)
        if (center_dist > 1e-6) {
            double theta = asin(averate_r / 1000.0 / center_dist) * config_.outpost_yaw_percent;
            double target_yaw = atan2(armors_xyz[index].y, armors_xyz[index].x);
            flag = std::fabs(limit_rad(target_yaw - theta_relative)) < theta;
            // std::cout << "[3板] 角度限制theta: " << theta << ", 目标yaw: " << target_yaw << ", flag: " << flag << std::endl;
        } else {
            flag = false;
        }
        ts.message.fire_allowance = flag;
        return index;
    }
    else {
        double angle[4];
        
        // 使用 x(0), x(2) 作为基准位置，而不是 center
        // if (ts.message.wheel_w != 0)
        // {
        //     float v_line = ts.message.wheel_w * ts.message.wheel_r * 0.2; // 轮腿线速度 mm/s
        // float v_x = x(1) - v_line * std::cos(ts.message.wheel_angle + M_PI / 2);
        // float v_y = x(3) - v_line * std::sin(ts.message.wheel_angle + M_PI / 2);
        // float v_x1 = v_line * std::cos(ts.message.wheel_angle + M_PI / 2 + ts.message.wheel_w * (ts.message.bias + latency));
        // float v_y1 = v_line * std::sin(ts.message.wheel_angle + M_PI / 2 + ts.message.wheel_w * (ts.message.bias + latency));
            
        //     for (int i = 0; i < 4; i++) {
        //         dt = time[i] + latency;
        //         // center.x = x(0) + v_x * dt + ts.message.wheel_r * std::cos(ts.message.wheel_angle + ts.message.wheel_w * (ts.message.bias / 2 + latency)) - ts.message.wheel_r * std::cos(ts.message.wheel_angle);
        //         // center.y = x(2) + v_y * dt + ts.message.wheel_r * std::sin(ts.message.wheel_angle + ts.message.wheel_w * (ts.message.bias / 2 + latency)) - ts.message.wheel_r * std::sin(ts.message.wheel_angle);
        //         center.x = x(0) + v_x * dt - ts.message.wheel_r * std::cos(ts.message.wheel_angle);
        // // center.y = x(2) + v_y * total_t + ts.message.wheel_r * std::sin(ts.message.wheel_angle + ts.message.wheel_w * (ts.message.bias + latency)) - ts.message.wheel_r * std::sin(ts.message.wheel_angle)
        // //         + v_y1 * (time - ts.message.bias);
        //         center.y = x(2) + v_y * dt - ts.message.wheel_r * std::sin(ts.message.wheel_angle);
        //         center.z = 0;
        //         angle[i] = x(9) + x(10) * dt - atan2(center.y, center.x) + M_PI * i / 2;
        //     }
        // }
        // else {
        dt = time[0] + latency;
        angle[0] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt);
    
        dt = time[1] + latency;
        angle[1] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt) + M_PI / 2;
            
        dt = time[2] + latency;
        angle[2] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt) + M_PI;
            
        dt = time[3] + latency;
        angle[3] = x(9) + x(10) * dt - atan2(x(2) + x(3) * dt, x(0) + x(1) * dt) + M_PI * 3 / 2;
            
        // }
        double spin_weight = config_.weight * x(10);
        
        // cout << "spin weight: " << spin_weight << endl;
        

        for (int i = 0; i < 4; i ++) {
            // cout << "angles" << i << ": " << angle[i] << endl;
            
            angle[i] -= spin_weight;
            angle[i] = atan2(sin(angle[i]), cos(angle[i]));
            // cout << "culculated angles" << i << ": " << angle[i] << endl;
        }
        // 已经计算并输出角度，设置标记
        // angles_valid_ = true;
        // --- NEW: 使用角度绝对值过滤 ---
        // 1. 创建索引列表并按角度绝对值排序
        std::vector<int> candidates = {0, 1, 2, 3};
        std::sort(candidates.begin(), candidates.end(), [&](int a, int b)
                  { return std::fabs(angle[a]) < std::fabs(angle[b]); });
        // 2. 只保留角度最小的两个装甲板
        candidates.resize(2);

        // 3. 在候选装甲板中寻找角度最小的，角度相近时优先选择上次选择的装甲板
        const double ANGLE_SIMILAR_THRESHOLD = 0.1;  // 角度相近阈值（约5.7度）
        int index = candidates[0]; // 默认选排序第一的（角度最小）

        // 如果第一名为上次选中的，它已经是index了，不用变
        // 如果第二名和第一名角度相差不大，且第二名是上次选中的，则改选第二名
        if (std::fabs(std::fabs(angle[candidates[1]]) - std::fabs(angle[candidates[0]])) < ANGLE_SIMILAR_THRESHOLD) {
            if (candidates[1] == index_temp_last) {
                index = candidates[1];
            }
        }
        
        // 更新上次选择状态
        index_temp_last = index; 
        
        // 4. 计算角度限制并设置flag
        dt = time[index] + latency;
        float theta_relative = atan2(x(2) + x(3) * dt, x(0) + x(1) * dt);
        double center_dist = sqrt(pow(center.x, 2) + pow(center.y, 2)) / 1000.0;  // 中心距离(m)
        if (center_dist > 1e-6) {
            double theta = asin(averate_r / 1000.0 / center_dist) * config_.yaw_percent;
            double target_yaw = atan2(armors_xyz[index].y, armors_xyz[index].x);
            flag = target_yaw > theta_relative - theta && target_yaw < theta + theta_relative;
            // std::cout << "[4板] 角度限制theta: " << theta << ", 目标yaw: " << target_yaw << ", flag: " << flag << std::endl;
        } else {
            flag = false;
        }
        ts.message.fire_allowance = flag;
        // cout << "aim angle: " << angle[index] << endl;
        return index;
    }
}

void Planner::setTargetState(const Eigen::VectorXd& x, int num) {
    EKF_statue = x;
    number = num;
}
