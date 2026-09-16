#include "SerialPort.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace {

bool baudRateToSpeed(int baud_rate, speed_t& speed) noexcept {
    switch (baud_rate) {
    case 300:
        speed = B300;
        return true;
    case 1200:
        speed = B1200;
        return true;
    case 2400:
        speed = B2400;
        return true;
    case 4800:
        speed = B4800;
        return true;
    case 9600:
        speed = B9600;
        return true;
    case 19200:
        speed = B19200;
        return true;
    case 38400:
        speed = B38400;
        return true;
    case 57600:
        speed = B57600;
        return true;
    case 115200:
        speed = B115200;
        return true;
#ifdef B230400
    case 230400:
        speed = B230400;
        return true;
#endif
    default:
        errno = EINVAL;
        return false;
    }
}

int remainingMilliseconds(
    const std::chrono::steady_clock::time_point deadline) noexcept {
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) {
        return 0;
    }
    return static_cast<int>(std::min<std::int64_t>(remaining.count(), 0x7fffffff));
}

int waitForFd(int fd, short events,
              const std::chrono::steady_clock::time_point deadline) noexcept {
    while (true) {
        const int timeout_ms = remainingMilliseconds(deadline);
        if (timeout_ms == 0) {
            return 0;
        }

        pollfd descriptor{fd, events, 0};
        const int result = ::poll(&descriptor, 1, timeout_ms);
        if (result < 0 && errno == EINTR) {
            continue;
        }
        if (result <= 0) {
            return result;
        }
        if ((descriptor.revents & events) != 0) {
            return 1;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            errno = EIO;
            return -1;
        }
    }
}

} // namespace

SerialPort::SerialPort(const std::string& device_name) noexcept {
    openDevice(device_name.c_str());
}

SerialPort::SerialPort(const char* device_name) noexcept {
    openDevice(device_name);
}

SerialPort::~SerialPort() {
    CloseSerialPort();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(std::exchange(other.fd_, -1)),
      pending_read_(std::move(other.pending_read_)),
      read_timeout_(other.read_timeout_),
      write_timeout_(other.write_timeout_) {}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this == &other) {
        return *this;
    }

    CloseSerialPort();
    fd_ = std::exchange(other.fd_, -1);
    pending_read_ = std::move(other.pending_read_);
    read_timeout_ = other.read_timeout_;
    write_timeout_ = other.write_timeout_;
    return *this;
}

bool SerialPort::openDevice(const char* device_name) noexcept {
    if (device_name == nullptr || device_name[0] == '\0') {
        errno = EINVAL;
        return false;
    }

    // Do not become the controlling terminal; poll() below handles blocking.
    const int opened_fd =
        ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (opened_fd < 0) {
        return false;
    }

    CloseSerialPort();
    fd_ = opened_fd;
    pending_read_.clear();
    return true;
}

bool SerialPort::InitSerialPort(int baud_rate,
                                int data_bits,
                                int stop_bits,
                                int parity_bit) noexcept {
    if (!isOpen()) {
        errno = EBADF;
        return false;
    }

    speed_t speed{};
    if (!baudRateToSpeed(baud_rate, speed)) {
        return false;
    }

    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {
        return false;
    }

    // Raw mode preserves binary frame bytes. CLOCAL ignores modem-control
    // lines and CREAD enables input; the default arguments select 8N1.
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= static_cast<tcflag_t>(~static_cast<tcflag_t>(CSIZE));
    switch (data_bits) {
    case 5:
        settings.c_cflag |= CS5;
        break;
    case 6:
        settings.c_cflag |= CS6;
        break;
    case 7:
        settings.c_cflag |= CS7;
        break;
    case 8:
        settings.c_cflag |= CS8;
        break;
    default:
        errno = EINVAL;
        return false;
    }

    if (stop_bits == 1) {
        settings.c_cflag &= static_cast<tcflag_t>(~static_cast<tcflag_t>(CSTOPB));
    } else if (stop_bits == 2) {
        settings.c_cflag |= CSTOPB;
    } else {
        errno = EINVAL;
        return false;
    }

    switch (parity_bit) {
    case 'n':
    case 'N':
        settings.c_cflag &= static_cast<tcflag_t>(~static_cast<tcflag_t>(PARENB));
        settings.c_iflag &= static_cast<tcflag_t>(~static_cast<tcflag_t>(INPCK));
        break;
    case 'o':
    case 'O':
        settings.c_cflag |= PARENB | PARODD;
        settings.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E':
        settings.c_cflag |= PARENB;
        settings.c_cflag &= static_cast<tcflag_t>(~static_cast<tcflag_t>(PARODD));
        settings.c_iflag |= INPCK;
        break;
    default:
        errno = EINVAL;
        return false;
    }

#ifdef CRTSCTS
    settings.c_cflag &= ~CRTSCTS;
#endif
    // This protocol uses no hardware or software flow control. VMIN/VTIME stay
    // zero because poll() provides the explicit Read/Write deadlines.
    const auto software_flow_control =
        static_cast<tcflag_t>(IXON | IXOFF | IXANY);
    settings.c_iflag &= static_cast<tcflag_t>(~software_flow_control);
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;

    if (::cfsetispeed(&settings, speed) != 0 ||
        ::cfsetospeed(&settings, speed) != 0) {
        return false;
    }
    if (::tcflush(fd_, TCIOFLUSH) != 0) {
        return false;
    }
    if (::tcsetattr(fd_, TCSANOW, &settings) != 0) {
        return false;
    }

    pending_read_.clear();
    return true;
}

bool SerialPort::CloseSerialPort() noexcept {
    pending_read_.clear();
    if (!isOpen()) {
        return true;
    }

    const int descriptor = std::exchange(fd_, -1);
    return ::close(descriptor) == 0;
}

int SerialPort::Read(char* buffer, int length) noexcept {
    if (!isOpen() || buffer == nullptr || length <= 0) {
        errno = !isOpen() ? EBADF : EINVAL;
        return -1;
    }

    const auto requested = static_cast<std::size_t>(length);
    const auto deadline = std::chrono::steady_clock::now() + read_timeout_;
    // Serial reads may return any prefix. Keep it across timeout boundaries and
    // publish data to the caller only when the requested byte count is complete.
    while (pending_read_.size() < requested) {
        const int ready = waitForFd(fd_, POLLIN, deadline);
        if (ready == 0) {
            return 0;
        }
        if (ready < 0) {
            return -1;
        }

        std::uint8_t chunk[256];
        const std::size_t wanted =
            std::min(sizeof(chunk), requested - pending_read_.size());
        const ssize_t count = ::read(fd_, chunk, wanted);
        if (count > 0) {
            pending_read_.insert(pending_read_.end(), chunk, chunk + count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            continue;
        }
        if (count == 0) {
            errno = EIO;
        }
        return -1;
    }

    std::memcpy(buffer, pending_read_.data(), requested);
    pending_read_.erase(pending_read_.begin(),
                        pending_read_.begin() + static_cast<std::ptrdiff_t>(requested));
    return length;
}

int SerialPort::Write(const char* buffer, int length) noexcept {
    if (!isOpen() || buffer == nullptr || length <= 0) {
        errno = !isOpen() ? EBADF : EINVAL;
        return -1;
    }

    const auto requested = static_cast<std::size_t>(length);
    std::size_t written = 0;
    const auto deadline = std::chrono::steady_clock::now() + write_timeout_;
    while (written < requested) {
        const ssize_t count = ::write(fd_, buffer + written, requested - written);
        if (count > 0) {
            written += static_cast<std::size_t>(count);
            continue;
        }
        if (count < 0 && errno == EINTR) {
            continue;
        }
        if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            return -1;
        }

        const int ready = waitForFd(fd_, POLLOUT, deadline);
        if (ready > 0) {
            continue;
        }
        if (ready == 0) {
            errno = ETIMEDOUT;
        }
        return -1;
    }

    return length;
}

void SerialPort::setReadTimeout(std::chrono::milliseconds timeout) noexcept {
    read_timeout_ = std::max(timeout, std::chrono::milliseconds{1});
}

void SerialPort::setWriteTimeout(std::chrono::milliseconds timeout) noexcept {
    write_timeout_ = std::max(timeout, std::chrono::milliseconds{1});
}

bool SerialPort::isOpen() const noexcept {
    return fd_ >= 0;
}

int SerialPort::ReceiveFd() const noexcept {
    return fd_;
}
