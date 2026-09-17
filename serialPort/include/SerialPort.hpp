#ifndef RMV_TRAINING_SERIAL_PORT_HPP
#define RMV_TRAINING_SERIAL_PORT_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

// 管理一个 Linux 串口文件描述符。带设备名的构造函数只负责打开设备；
// 执行 I/O 前还需调用 InitSerialPort()。打开或配置失败时通过 false/isOpen()
// 报告；析构时始终关闭仍处于打开状态的设备。
class SerialPort {
public:
    SerialPort() noexcept = default;
    explicit SerialPort(const std::string& device_name) noexcept;
    explicit SerialPort(const char* device_name) noexcept;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    bool InitSerialPort(int baud_rate,
                        int data_bits = 8,
                        int stop_bits = 1,
                        int parity_bit = 'N') noexcept;
    bool CloseSerialPort() noexcept;

    // 只有在所需字节数全部可用时才返回长度；超时返回 0，出错返回 -1。
    // 短读数据会缓存到下一次调用，因此调用方不会收到不完整的请求缓冲区。
    int Read(char* buffer, int length) noexcept;

    // 持续写入直到完整缓冲区发送完毕。成功返回 length，出错或超时返回 -1；
    // 返回 -1 时，前缀数据可能已经发送到串口。
    int Write(const char* buffer, int length) noexcept;

    // 两种等待超时默认均为 200 ms；非正值会限制为 1 ms。
    void setReadTimeout(std::chrono::milliseconds timeout) noexcept;
    void setWriteTimeout(std::chrono::milliseconds timeout) noexcept;

    [[nodiscard]] bool isOpen() const noexcept;
    [[nodiscard]] int ReceiveFd() const noexcept;

private:
    bool openDevice(const char* device_name) noexcept;

    int fd_{-1};
    std::vector<std::uint8_t> pending_read_;
    std::chrono::milliseconds read_timeout_{200};
    std::chrono::milliseconds write_timeout_{200};
};

#endif
