#ifndef PLANNER
#define PLANNER
#include <Eigen/Dense>
#include <list>
#include <optional>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>

// #include "../tinympc/tiny_api.hpp"
#include "BulletModel.hpp"
#include "tracker.hpp"
#include "globalParam.hpp"

// 配置参数结构体
struct PlannerConfig {
    double dt = 0.02;                    // 时间步长
    double yaw_percent = 0.5;            // 普通车辆开火角度比例
    double outpost_yaw_percent = 0.5;    // 前哨站开火角度比例
    // int horizon = 100;                   // 预测时域
    // int half_horizon = 50;               // 半时域
    float latency = 0.1;                        // 时间延迟

    double weight = - 0.01;                //  经验系数

    // 控制参数
    double yaw_offset = 0.0;             // 偏航补偿
    double pitch_offset = 0.0;           // 俯仰补偿
    double fire_thresh = 0.05;           // 开火阈值（弧度）

    // 延迟参数
    double decision_speed = 2.0;         // 速度决策阈值
    double high_speed_delay_time = 0.1;  // 高速延迟
    double low_speed_delay_time = 0.05;  // 低速延迟

    // MPC权重
    std::vector<double> Q_yaw = {80.0, 60.0};   // yaw状态权重
    std::vector<double> R_yaw = {3.0};   // yaw控制权重
    std::vector<double> Q_pitch = {80.0, 60.0}; // pitch状态权重
    std::vector<double> R_pitch = {5.0};  // pitch控制权重

    // 约束
    double max_yaw_acc = 50.0;           // 最大偏航角加速度 rad/s²
    double max_pitch_acc = 30.0;         // 最大俯仰角加速度 rad/s²

    // 子弹参数
    double bullet_speed = 23.5;          // 默认子弹速度 m/s
    double min_bullet_speed = 10.0;      // 最小子弹速度
    double max_bullet_speed = 25.0;      // 最大子弹速度
    
    // 前哨站参数 (根据RoboMaster规格图纸)
    double outpost_r = 275.0;            // 前哨站转轴半径 (mm)
    double outpost_z_offset_0 = 0.0;     // 装甲板0的z偏移 (mm)
    double outpost_z_offset_1 = -102.0;  // 装甲板1的z偏移 (mm)
    double outpost_z_offset_2 = 102.0;   // 装甲板2的z偏移 (mm)

    // 弹道参数
//    double gravity = 9.7833;             // 重力加速度 m/s²
};

struct Plan
{
  bool control;      // 是否进行控制
//  bool fire;         // 是否开火
  float target_yaw;  // 目标偏航角
  float target_pitch;// 目标俯仰角
  float yaw;         // 当前偏航角
  float yaw_vel;     // 当前偏航角速度
  float yaw_acc;     // 当前偏航角加速度
  float pitch;       // 当前俯仰角
  float pitch_vel;   // 当前俯仰角速度
  float pitch_acc;   // 当前俯仰角加速度
};

class Planner
{
public:
    std::deque<double> yaw_history_;
    const size_t HISTORY_SIZE = 10;  // 历史窗口大小
    const double YAW_JUMP_THRESHOLD = 0.8;  // yaw跳变阈值

    Eigen::Vector4d debug_xyza;
    explicit Planner(const PlannerConfig& config = PlannerConfig());
    ~Planner();

    Planner(const PlannerConfig &config, const Eigen::VectorXd& x, int num);

    // 配置相关
    void setConfig(const PlannerConfig& config);
    PlannerConfig getConfig() const;

    // 主规划函数
//    Plan plan(const std::vector<Armor>& detected_armors, const Translator& ts, double dt);
    // 单个目标规划
    Plan plan(double bullet_speed, Translator &ts, float dt);
    // 获取调试信息
    Eigen::Vector4d getDebugInfo() const { return debug_xyza; }
    void setTargetState(const Eigen::VectorXd& x, int num);

    // void setupYawSolver(float dt);
    // void setupPitchSolver(float dt);
    

private:
    // 改用全局变量
    int target_armor_index = -1;
    bool armor_switching_ = false;

    PlannerConfig config_;
    //Armor shot_armor_;
    //Armor predict_armor_;
    Eigen::VectorXd EKF_statue; // 当前帧的EKF状态量
    int number;  // 装甲板的数字，考虑前哨战
    cv::Point3f center; // 对方车中心位置在车坐标系下的坐标
    cv::Point3f armors_xyz[4];
    Eigen::Vector3f translation_vector{74.1, 0.0, 0.0};//枪管坐标系相对车体坐标系的平移向量
    // Eigen::Vector3f translation_vector{0.0, 0.0, 0.0};//枪管坐标系相对车体坐标系的平移向量
    float time[4] = {0.0f};
    int index_temp = 0;
    int index_temp_last = 0;
    int index_ref[2] = {0};  // 参考装甲板编号
    int index_index = 0;  // index_ref的写指针
    int index_last_queue[10] = {0};  // 历史装甲板编号队列
    float t = 0.0f;
    float t_last = 0.0f;
    Eigen::Vector3f aim_armor_coo{0.0, 0.0, 0.0};
    bool flag = false;  // 判断yaw角范围
    double averate_r = 0.0;

    int8_t i_in = 0;
    uint16_t index_cnt = 0;  // 当前装甲板连续稳定计数
    uint8_t aim_num_fault_flag = 0;  // 装甲板选择故障标志位

    // 弹道计算
//    Trajectory calculateTrajectory(double bullet_speed, double distance, double height) const;
    // Eigen::Matrix<double, 2, 1> iterationPredict(double bullet_speed);

    Eigen::Vector2d getTargetYawPitch(double bullet_speed, Translator &ts);

//    std::vector<cv::Point3f>
//    predictArmorsAtTime(double t, const Eigen::VectorXd& x);

    // std::vector<cv::Point3f> predictArmorsAtTime(double t, const Eigen::VectorXd& x) const;



   void setEKFState(Eigen::VectorXd& X);  // 设置状态量
   void trackLine(float time, Eigen::VectorXd& x, Translator &ts, int target_idx = -1);  // 求装甲板轨迹
    void transform(int armor_num, float pitch_angle, float yaw_angle);
   double calculateTargetYawVelocity(int armor_num, float predict_time, const Eigen::VectorXd &x, const Translator &ts) const;
   int SelectShoot(Eigen::VectorXd &x, Translator &ts);             // 选择要打的装甲板

public:
   // 获取trackLine计算出的预测装甲板世界坐标
   int getTargetArmorIndex() const { return target_armor_index; }
   void getTrackArmorsXYZ(std::vector<cv::Point3f> &points, std::vector<double> &yaws);
   int getArmorNumber() const { return number; }

    static inline double limit_rad(double angle)
    {
        while (angle > M_PI)  angle -= 2.0 * M_PI;
        while (angle <= -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

#endif // PLANNER
