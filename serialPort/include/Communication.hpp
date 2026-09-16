#ifndef RMV_TRAINING_COMMUNICATION_HPP
#define RMV_TRAINING_COMMUNICATION_HPP

#include <cstddef>
#include <cstdint>
#include <type_traits>

inline constexpr std::uint8_t kFrameHead = 0x71;
inline constexpr std::uint8_t kFrameTail = 0x4C;
inline constexpr std::size_t kCommunicationFrameSize = 64;

// This is the wire layout, not an ordinary in-memory model. One-byte packing
// removes compiler padding; changing any field breaks compatibility with MCU.
#pragma pack(push, 1)
struct MessData_AutoAim {
    std::uint8_t head; // Both sides: frame head, fixed 0x71.

    float yaw;          // MCU -> vision: current gimbal yaw, rad.
    float pitch;        // MCU -> vision: current gimbal pitch, rad.
    float roll;         // MCU -> vision: current gimbal roll, rad.
    std::uint8_t status; // MCU -> vision: team/mode; this task accepts 0 or 5.
    std::uint8_t is_far; // MCU -> vision: legacy range flag; unused in this task.
    std::uint8_t armor_flag; // Vision -> MCU: robot ID; 0 means no target.
    float latency;      // Vision -> MCU: processing latency, ms.
    float bias;         // MCU -> vision: prediction time bias, s.
    float distance;     // Vision -> MCU: target distance, m.
    float pitch_offset; // Reserved legacy field; do not repurpose.

    float coo_x; // Vision -> MCU: EKF vehicle-center x, mm.
    float coo_y; // Vision -> MCU: EKF vehicle-center y, mm.
    float wheel_w; // Reserved legacy field; do not repurpose.
    std::uint8_t fire_allowance; // Vision -> MCU: 0 inhibit, 1 permit fire.

    float target_yaw;   // Vision -> MCU: commanded yaw, rad.
    float target_pitch; // Vision -> MCU: commanded pitch, rad.
    float yaw_vel;      // Vision -> MCU: target yaw rate, rad/s.
    float pitch_vel;    // Vision -> MCU: target pitch rate, rad/s.
    std::uint16_t crc;  // Vision -> MCU: EKF state 0/1/2, not a checksum.
    std::uint8_t tail;  // Both sides: frame tail, fixed 0x4C.
};
#pragma pack(pop)

// The union exposes the same 64 bytes as named fields or a serial buffer.
union Translator {
    MessData_AutoAim message;
    char data[kCommunicationFrameSize];
};

// Size and offset checks make accidental protocol layout changes fail at build.
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
