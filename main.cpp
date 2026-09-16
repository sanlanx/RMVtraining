#include "AppSupport.hpp"
#include "SerialPort.hpp"
#include "StudentTasks.hpp"
#include "WebLogger.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kDefaultWebPort = 8080;
// Avoid a busy loop while waiting for a usable controller message or camera frame.
constexpr auto kIdleDelay = std::chrono::milliseconds(10);
// Commands derived from an older controller snapshot must never permit firing.
constexpr auto kMessageMaxAge = std::chrono::milliseconds(500);

std::atomic<bool> g_running{true};

struct SharedMessage {
    // This is a latest-value mailbox, not a queue. The mutex protects all fields below.
    std::mutex mutex;
    Translator latest{};
    std::chrono::steady_clock::time_point received_at{};
    bool available{false};
};

struct AppContext {
    SerialPort& serial;
    SharedMessage& shared;
    const CameraParameters& camera;
};

struct Options {
    std::string serial_device;
    int baud_rate{115200};
    std::string camera_config;
    int web_port{kDefaultWebPort};
};

void handleSignal(int) {
    g_running.store(false);
}

void printUsage(const char* program) {
    std::cout << "Usage: " << program
              << " <serial_device> <baud_rate> <camera_config> [web_port]\n"
              << "Example: " << program
              << " /dev/ttyACM0 115200 config/camera.yaml 8080\n";
}

bool parsePositiveInt(const char* text, int& value) {
    try {
        std::size_t parsed = 0;
        const int candidate = std::stoi(text, &parsed);
        if (text[parsed] != '\0' || candidate <= 0) {
            return false;
        }
        value = candidate;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

bool parseOptions(int argc, char** argv, Options& options) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        printUsage(argv[0]);
        return false;
    }
    if (argc < 4 || argc > 5) {
        printUsage(argv[0]);
        return false;
    }

    options.serial_device = argv[1];
    options.camera_config = argv[3];
    if (!parsePositiveInt(argv[2], options.baud_rate)) {
        std::cerr << "Invalid baud rate: " << argv[2] << '\n';
        return false;
    }
    if (argc == 5 && !parsePositiveInt(argv[4], options.web_port)) {
        std::cerr << "Invalid web port: " << argv[4] << '\n';
        return false;
    }
    return true;
}

bool copyLatestMessage(SharedMessage& shared, Translator& destination, bool& fresh,
                       std::chrono::steady_clock::time_point& received_at) {
    // Copy the frame and its timestamp under one lock so freshness refers to this snapshot.
    std::lock_guard<std::mutex> lock(shared.mutex);
    if (!shared.available) {
        return false;
    }
    destination = shared.latest;
    received_at = shared.received_at;
    fresh = std::chrono::steady_clock::now() - received_at <= kMessageMaxAge;
    return true;
}

bool hasFrameMarkers(const char* frame) noexcept {
    // Marker validation establishes framing only; payload values are checked by the worker.
    return static_cast<std::uint8_t>(frame[0]) == kFrameHead &&
           static_cast<std::uint8_t>(frame[kCommunicationFrameSize - 1]) == kFrameTail;
}

bool hasFiniteControlInput(const MessData_AutoAim& message) noexcept {
    return std::isfinite(message.yaw) && std::isfinite(message.pitch) &&
           std::isfinite(message.roll) && std::isfinite(message.bias);
}

bool sendReply(SerialPort& serial, Translator& translator) {
    // The wire protocol is fixed-size: a partial write is treated as a fatal link error.
    translator.message.head = kFrameHead;
    translator.message.tail = kFrameTail;
    const int written = serial.Write(translator.data, static_cast<int>(sizeof(translator.data)));
    if (written != static_cast<int>(sizeof(translator.data))) {
        std::cerr << "Serial write failed\n";
        g_running.store(false);
        return false;
    }
    return true;
}

void logPrediction(const PredictionResult& result) {
    WEB_LOG("target_found", result.state == TrackingState::NoTarget ? 0.0 : 1.0);
    if (result.state == TrackingState::NoTarget) {
        return;
    }
    WEB_LOG("target_id", result.target_id);
    WEB_LOG("target_yaw", result.target_yaw);
    WEB_LOG("target_pitch", result.target_pitch);
    WEB_LOG("target_distance", result.distance_m);
    WEB_LOG("ekf_x_m", result.center_x_m);
    WEB_LOG("ekf_y_m", result.center_y_m);
    WEB_LOG("ekf_vx_m_s", result.center_velocity_x_m_s);
    WEB_LOG("ekf_vy_m_s", result.center_velocity_y_m_s);
    WEB_LOG("ekf_yaw_rad", result.body_yaw);
    WEB_LOG("ekf_yaw_rate_rad_s", result.body_yaw_velocity);
    WEB_LOG("ekf_radius_1_m", result.radius_1_m);
    WEB_LOG("ekf_radius_2_m", result.radius_2_m);
}

}  // namespace

// Serial receiver: reconstruct wire frames and publish only the newest snapshot.
void ReadFunction(AppContext& context) {
    // Read() returns fixed-size chunks. Keep the trailing bytes between calls so the
    // parser can recover if the process starts mid-frame or one byte is inserted/lost.
    std::vector<char> stream_buffer;
    stream_buffer.reserve(kCommunicationFrameSize * 2);

    while (g_running.load()) {
        std::array<char, kCommunicationFrameSize> chunk{};
        const int count =
            context.serial.Read(chunk.data(), static_cast<int>(chunk.size()));
        if (count < 0) {
            std::cerr << "Serial read failed\n";
            g_running.store(false);
            break;
        }
        if (count == 0) {
            continue;
        }
        if (count > static_cast<int>(chunk.size())) {
            continue;
        }

        stream_buffer.insert(stream_buffer.end(), chunk.begin(), chunk.begin() + count);
        std::size_t consumed = 0;
        while (stream_buffer.size() - consumed >= kCommunicationFrameSize) {
            const char* candidate = stream_buffer.data() + consumed;
            if (!hasFrameMarkers(candidate)) {
                // Shift by one byte until a complete 0x71 ... 0x4C frame is found.
                ++consumed;
                continue;
            }

            Translator incoming{};
            std::memcpy(&incoming.message, candidate, sizeof(incoming.message));
            consumed += kCommunicationFrameSize;

            // Publish structurally complete frames even when their payload is invalid.
            // The operation thread can then send a safe reply immediately.
            std::lock_guard<std::mutex> lock(context.shared.mutex);
            context.shared.latest = incoming;
            context.shared.received_at = std::chrono::steady_clock::now();
            context.shared.available = true;
        }

        if (consumed != 0U) {
            stream_buffer.erase(stream_buffer.begin(), stream_buffer.begin() + consumed);
        }
    }
}

// Vision worker: acquire, detect, solve, predict, validate and send one reply.
void OperationFunction(AppContext& context) {
    auto last_frame_time = std::chrono::steady_clock::time_point{};

    while (g_running.load()) {
        Translator output{};
        bool message_is_fresh = false;
        std::chrono::steady_clock::time_point message_received_at{};
        if (!copyLatestMessage(context.shared, output, message_is_fresh,
                               message_received_at)) {
            std::this_thread::sleep_for(kIdleDelay);
            continue;
        }
        // Stale data, unsupported modes and invalid controller values all take
        // the same no-target path; no student algorithm is called in these cases.
        if (!message_is_fresh) {
            setSafeOutput(output);
            sendReply(context.serial, output);
            std::this_thread::sleep_for(kIdleDelay);
            continue;
        }
        if (!isAutoAimStatus(output.message.status) ||
            !hasFiniteControlInput(output.message)) {
            setSafeOutput(output);
            sendReply(context.serial, output);
            std::this_thread::sleep_for(kIdleDelay);
            continue;
        }

        const auto process_start = std::chrono::steady_clock::now();
        const GimbalState gimbal = gimbalStateFromMessage(output.message);
        cv::Mat pic;
        const auto finishFrame = [&](const PredictionResult& logged_result) {
            const auto process_end = std::chrono::steady_clock::now();
            PredictionResult result_to_log = logged_result;
            // Student algorithms may block. Recheck the exact input snapshot before
            // writing so a result that became stale during processing is made safe.
            if (process_end - message_received_at > kMessageMaxAge) {
                setSafeOutput(output);
                result_to_log = {};
            }
            output.message.latency = static_cast<float>(std::chrono::duration<double, std::milli>(
                                                            process_end - process_start)
                                                            .count());
            WEB_LOG("process_time_ms", output.message.latency);
            logPrediction(result_to_log);
            return sendReply(context.serial, output);
        };

        // Student work starts at image acquisition; all failure guards remain provided.
        if (!get_pic(pic) || pic.empty()) {
            setSafeOutput(output);
            finishFrame({});
            std::this_thread::sleep_for(kIdleDelay);
            continue;
        }

        const auto frame_time = std::chrono::steady_clock::now();
        if (last_frame_time.time_since_epoch().count() != 0) {
            const double frame_seconds =
                std::chrono::duration<double>(frame_time - last_frame_time).count();
            if (frame_seconds > 0.0) {
                WEB_LOG("camera_fps", 1.0 / frame_seconds);
            }
        }
        last_frame_time = frame_time;

        const auto detections = armor_detect(pic, enemyColorFromStatus(output.message.status));
        WEB_LOG("armor_count", static_cast<double>(detections.size()));
        if (detections.empty()) {
            setSafeOutput(output);
            finishFrame({});
            continue;
        }

        const auto& first_detection = detections.front();
        // The first target is logged only for visualization; selection belongs to EKF.
        [[maybe_unused]] const cv::Point2f center =
            (first_detection.corners[0] + first_detection.corners[1] +
             first_detection.corners[2] + first_detection.corners[3]) *
            0.25F;
        WEB_LOG("target_x_px", center.x);
        WEB_LOG("target_y_px", center.y);

        const auto observations = armor_solve(detections, context.camera, gimbal);
        if (observations.empty()) {
            setSafeOutput(output);
            finishFrame({});
            continue;
        }
        WEB_LOG("pnp_reprojection_error", observations.front().reprojection_error);

        const double timestamp_seconds =
            std::chrono::duration<double>(frame_time.time_since_epoch()).count();
        // steady_clock supplies a monotonic timestamp suitable for EKF delta-time.
        const PredictionResult prediction =
            ekf_predict(observations, gimbal, timestamp_seconds);
        const bool valid_prediction = applyPrediction(output, prediction);
        finishFrame(valid_prediction ? prediction : PredictionResult{});
    }
}

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        return argc == 2 && std::string(argv[1]) == "--help" ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    CameraParameters camera;
    std::string camera_error;
    if (!loadCameraParameters(options.camera_config, camera, camera_error)) {
        std::cerr << camera_error << '\n';
        return EXIT_FAILURE;
    }

    SerialPort serial(options.serial_device);
    if (!serial.isOpen()) {
        std::cerr << "Cannot open serial device: " << options.serial_device << '\n';
        return EXIT_FAILURE;
    }
    if (!serial.InitSerialPort(options.baud_rate, 8, 1, 'N')) {
        std::cerr << "Cannot configure serial device\n";
        return EXIT_FAILURE;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    START_WEB_SERVER(options.web_port);

    SharedMessage shared;
    AppContext context{serial, shared, camera};
    // ReadFunction only publishes controller frames. OperationFunction owns the
    // camera/algorithm pipeline and is the only thread that writes serial replies.
    std::thread read_thread(ReadFunction, std::ref(context));
    std::thread operation_thread(OperationFunction, std::ref(context));

    read_thread.join();
    g_running.store(false);
    operation_thread.join();
    // Stop the HTTP thread only after producers have stopped logging.
    STOP_WEB_SERVER();
    return EXIT_SUCCESS;
}
