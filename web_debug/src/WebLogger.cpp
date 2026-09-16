#include "WebLogger.hpp"

#ifdef WEB_DEBUG

#include "httplib.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>

namespace {

// Resume after the last event acknowledged by a reconnecting SSE client.
std::uint64_t parseLastEventId(const httplib::Request& request) {
    if (!request.has_header("Last-Event-ID")) {
        return 0;
    }

    try {
        const auto value = std::stoull(request.get_header_value("Last-Event-ID"));
        if (value == std::numeric_limits<std::uint64_t>::max()) {
            return 0;
        }
        return value + 1;
    } catch (...) {
        return 0;
    }
}

} // namespace

WebLogger& WebLogger::getInstance() {
    static WebLogger instance;
    return instance;
}

WebLogger::WebLogger() = default;

WebLogger::~WebLogger() {
    stop();
}

void WebLogger::start(int port, const std::string& host) {
    if (port <= 0 || port > 65535 || host.empty()) {
        std::cerr << "[WebLogger] Invalid listen address: " << host << ':' << port << '\n';
        return;
    }

    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (running_.load(std::memory_order_acquire)) {
        return;
    }

    // Join a previous or unexpectedly exited run before replacing its server.
    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    server_.reset();

    {
        std::lock_guard<std::mutex> buffer_lock(buffer_mutex_);
        buffer_.clear();
        next_sequence_ = 1;
    }

    // Register routes and bind first, so a bind failure creates no thread.
    auto server = std::make_unique<httplib::Server>();
    configureServer(*server);
    if (!server->bind_to_port(host, port)) {
        std::cerr << "[WebLogger] Failed to bind " << host << ':' << port << '\n';
        return;
    }

    server_ = std::move(server);
    running_.store(true, std::memory_order_release);
    server_thread_ = std::thread(&WebLogger::serverLoop, this);
    // Wait for the accept loop to remove the immediate start/stop race.
    server_->wait_until_ready();
    if (!server_->is_running()) {
        running_.store(false, std::memory_order_release);
        data_cv_.notify_all();
        server_thread_.join();
        server_.reset();
        std::cerr << "[WebLogger] HTTP server could not enter the listen loop\n";
        return;
    }
    std::cout << "[WebLogger] Dashboard: http://" << host << ':' << port << '\n';
}

void WebLogger::stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

    running_.store(false, std::memory_order_release);
    data_cv_.notify_all();

    if (server_) {
        // stop() closes an active listener. decommission() also covers a
        // listener that has not fully entered its accept loop.
        server_->stop();
        server_->decommission();
    }

    if (server_thread_.joinable()) {
        server_thread_.join();
    }
    server_.reset();
}

void WebLogger::log(const std::string& key, double value) {
    if (!running_.load(std::memory_order_acquire) || key.empty() || !std::isfinite(value)) {
        return;
    }

    const auto now = std::chrono::system_clock::now();
    const auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                               now.time_since_epoch())
                               .count();

    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!running_.load(std::memory_order_relaxed)) {
            return;
        }
        // Drop the oldest sample at capacity to keep producer cost bounded.
        if (buffer_.size() >= kBufferCapacity) {
            buffer_.pop_front();
        }
        buffer_.push_back({key, value, timestamp, next_sequence_++});
    }
    data_cv_.notify_all();
}

bool WebLogger::isRunning() const noexcept {
    return running_.load(std::memory_order_acquire);
}

nlohmann::json WebLogger::getBufferJSON() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    auto result = nlohmann::json::array();
    for (const auto& data : buffer_) {
        result.push_back({{"k", data.key}, {"v", data.value}, {"t", data.timestamp}});
    }
    return result;
}

void WebLogger::configureServer(httplib::Server& server) {
    server.Get("/health", [this](const httplib::Request&, httplib::Response& response) {
        response.set_header("Cache-Control", "no-store");
        response.set_content(
            running_.load(std::memory_order_acquire) ? "OK" : "STOPPING", "text/plain");
    });

    server.Get("/data", [this](const httplib::Request& request, httplib::Response& response) {
        // Each connection owns a cursor. Clients share history without
        // consuming samples from one another.
        std::uint64_t cursor = parseLastEventId(request);
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            if (cursor == 0) {
                cursor = buffer_.empty() ? next_sequence_ : buffer_.front().sequence;
            }
        }

        response.set_header("Cache-Control", "no-cache, no-transform");
        response.set_header("X-Accel-Buffering", "no");
        response.set_chunked_content_provider(
            "text/event-stream",
            [this, cursor](std::size_t, httplib::DataSink& sink) mutable {
                nlohmann::json batch = nlohmann::json::array();
                std::uint64_t last_sequence = 0;

                {
                    std::unique_lock<std::mutex> lock(buffer_mutex_);
                    data_cv_.wait_for(lock, std::chrono::seconds(5), [this, cursor] {
                        return !running_.load(std::memory_order_acquire) ||
                               (!buffer_.empty() && buffer_.back().sequence >= cursor);
                    });

                    if (!running_.load(std::memory_order_acquire)) {
                        sink.done();
                        return true;
                    }

                    // A client behind the bounded history resumes at its front.
                    if (!buffer_.empty() && cursor < buffer_.front().sequence) {
                        cursor = buffer_.front().sequence;
                    }
                    for (const auto& data : buffer_) {
                        if (data.sequence < cursor) {
                            continue;
                        }
                        batch.push_back(
                            {{"k", data.key}, {"v", data.value}, {"t", data.timestamp}});
                        last_sequence = data.sequence;
                    }
                    if (last_sequence != 0) {
                        cursor = last_sequence + 1;
                    }
                }

                // An SSE comment keeps an otherwise idle connection alive.
                const std::string payload = batch.empty()
                                                ? ": keep-alive\n\n"
                                                : "id: " + std::to_string(last_sequence) +
                                                      "\ndata: " + batch.dump() + "\n\n";
                if (!sink.is_writable()) {
                    return false;
                }
                return sink.write(payload.data(), payload.size());
            });
    });

    // HTML, styles, and plotting code are local, so the page works offline.
    bool static_mounted = false;
#ifdef WEB_DEBUG_STATIC_DIR
    static_mounted = server.set_mount_point("/", WEB_DEBUG_STATIC_DIR);
#else
    static_mounted = server.set_mount_point("/", "web_debug/static");
    if (!static_mounted) {
        static_mounted = server.set_mount_point("/", "../web_debug/static");
    }
#endif
    if (!static_mounted) {
        std::cerr << "[WebLogger] Static dashboard directory was not found\n";
        server.Get("/", [](const httplib::Request&, httplib::Response& response) {
            response.status = 503;
            response.set_content("Dashboard static files are unavailable.", "text/plain");
        });
    }
}

void WebLogger::serverLoop() {
    const bool listened = server_->listen_after_bind();
    const bool was_running = running_.exchange(false, std::memory_order_acq_rel);
    data_cv_.notify_all();
    if (!listened && was_running) {
        std::cerr << "[WebLogger] HTTP server stopped unexpectedly\n";
    }
}

#endif // WEB_DEBUG
