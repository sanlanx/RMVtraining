#ifndef RMV_TRAINING_COMMUNICATION_HPP
#define RMV_TRAINING_COMMUNICATION_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

inline constexpr std::uint8_t kFrameHead = 0x71;
inline constexpr std::uint8_t kFrameTail = 0x4C;
inline constexpr std::size_t kCommunicationFrameSize = 64;

// 这是串口线上的布局，而不是普通的内存模型。单字节对齐会移除编译器填充；
// 修改任意字段都会破坏与 MCU 的通信兼容性。
#pragma pack(push, 1)
struct MessData_AutoAim {
    std::uint8_t head; // 双方：帧头，固定为 0x71。

    float yaw;          // MCU -> 视觉：当前云台偏航角，单位 rad。
    float pitch;        // MCU -> 视觉：当前云台俯仰角，单位 rad。
    float roll;         // MCU -> 视觉：当前云台横滚角，单位 rad。
    std::uint8_t status; // MCU -> 视觉：队伍/模式；本任务只接受 0 或 5。
    std::uint8_t is_far; // MCU -> 视觉：历史距离标志；本任务不使用。
    std::uint8_t armor_flag; // 视觉 -> MCU：机器人编号；0 表示无目标。
    float latency;      // 视觉 -> MCU：处理延迟，单位 ms。
    float bias;         // MCU -> 视觉：预测时间偏置，单位 s。
    float distance;     // 视觉 -> MCU：目标距离，单位 m。
    float pitch_offset; // 历史保留字段；不要改作他用。

    float coo_x; // 视觉 -> MCU：EKF 估计的车体中心 x，单位 mm。
    float coo_y; // 视觉 -> MCU：EKF 估计的车体中心 y，单位 mm。
    float wheel_w; // 历史保留字段；不要改作他用。
    std::uint8_t fire_allowance; // 视觉 -> MCU：0 禁止开火，1 允许开火。

    float target_yaw;   // 视觉 -> MCU：指令偏航角，单位 rad。
    float target_pitch; // 视觉 -> MCU：指令俯仰角，单位 rad。
    float yaw_vel;      // 视觉 -> MCU：目标偏航角速度，单位 rad/s。
    float pitch_vel;    // 视觉 -> MCU：目标俯仰角速度，单位 rad/s。
    std::uint16_t crc;  // 视觉 -> MCU：EKF 状态 0/1/2，不是校验和。
    std::uint8_t tail;  // 双方：帧尾，固定为 0x4C。
};
#pragma pack(pop)

// 联合体让同一组 64 字节既可按字段访问，也可作为串口缓冲区使用。
union Translator {
    MessData_AutoAim message;
    char data[kCommunicationFrameSize];
};

// 大小和偏移检查可在编译阶段阻止意外的协议布局变化。
static_assert(sizeof(MessData_AutoAim) == kCommunicationFrameSize,
              "MessData_AutoAim must remain a 64-byte wire frame");
static_assert(sizeof(Translator) == kCommunicationFrameSize,
              "Translator must remain a 64-byte wire frame");
static_assert(std::is_standard_layout_v<MessData_AutoAim>);
static_assert(std::is_trivially_copyable_v<MessData_AutoAim>);
static_assert(offsetof(MessData_AutoAim, head) == 0);
static_assert(offsetof(MessData_AutoAim, yaw) == 1);
static_assert(offsetof(MessData_AutoAim, pitch) == 5);
static_assert(offsetof(MessData_AutoAim, roll) == 9);
static_assert(offsetof(MessData_AutoAim, status) == 13);
static_assert(offsetof(MessData_AutoAim, is_far) == 14);
static_assert(offsetof(MessData_AutoAim, armor_flag) == 15);
static_assert(offsetof(MessData_AutoAim, latency) == 16);
static_assert(offsetof(MessData_AutoAim, bias) == 20);
static_assert(offsetof(MessData_AutoAim, distance) == 24);
static_assert(offsetof(MessData_AutoAim, pitch_offset) == 28);
static_assert(offsetof(MessData_AutoAim, coo_x) == 32);
static_assert(offsetof(MessData_AutoAim, coo_y) == 36);
static_assert(offsetof(MessData_AutoAim, wheel_w) == 40);
static_assert(offsetof(MessData_AutoAim, fire_allowance) == 44);
static_assert(offsetof(MessData_AutoAim, target_yaw) == 45);
static_assert(offsetof(MessData_AutoAim, target_pitch) == 49);
static_assert(offsetof(MessData_AutoAim, yaw_vel) == 53);
static_assert(offsetof(MessData_AutoAim, pitch_vel) == 57);
static_assert(offsetof(MessData_AutoAim, crc) == 61);
static_assert(offsetof(MessData_AutoAim, tail) == 63);

#endif
