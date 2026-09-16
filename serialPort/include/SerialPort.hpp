#ifndef RMV_TRAINING_SERIAL_PORT_HPP
#define RMV_TRAINING_SERIAL_PORT_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

// Owns one Linux serial file descriptor. A constructor receiving a device name
// only opens it; call InitSerialPort() before I/O. Open/configuration failures
// are reported by false/isOpen(), and destruction always closes an open device.
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

    // Returns length only after that many bytes are available, 0 on timeout,
    // and -1 on error. A short read is cached for the next call, so callers
    // never receive a partial requested buffer.
    int Read(char* buffer, int length) noexcept;

    // Writes until the complete buffer is sent. Returns length on success and
    // -1 on error/timeout; after -1, a prefix may already be on the wire.
    int Write(const char* buffer, int length) noexcept;

    // Both waits default to 200 ms; non-positive values are clamped to 1 ms.
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
