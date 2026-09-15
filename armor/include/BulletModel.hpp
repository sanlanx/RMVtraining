#ifndef BULLETMODEL
#define BULLETMODEL


///尝试将弹道模型迭代自瞄pitch角度的变量归为一个类
///坐标系定义：与预测端采用同样的定义，前x，左y，上z
#define Kf 0.08f //实际上是空气阻力系数与小弹丸质量之比,与到处的变量名冲突最后选了这个
#define gravity 9.81f //重力加速度

class Bullet_Model {
public:
    Bullet_Model(float dist, float height);
    float bullet_iteration_cal(); // 迭代计算飞行时间的函数

    // 成员变量
    float dist_x; // 初始水平方向的距离
    float height_z; // 初始竖直方向的距离 
    float velo; // 子弹初速

    /// 取用iter表示迭代，后面变量用于迭代
    int iter_num; // 迭代次数
    float beta; // 弧度制下的角度
    float alpha_inter; // 用于计算时间的中间变量
    float err_max;
    float z_iter; // 迭代的竖直距离
    float z_iter_last; // 上次迭代的竖直距离
    float z_diff; // 某次迭代后的竖直距离与目标值的差值
    float flight_time;
    
    // 注释掉的pitch迭代函数声明
    // float pitch_iteration_cal(); // 迭代pitch角度的函数
};

#endif  // BULLETMODEL
