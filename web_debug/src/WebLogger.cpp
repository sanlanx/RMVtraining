#include "WebLogger.hpp"

#ifdef WEB_DEBUG

#include "../3rdparty/httplib.h"
#include <chrono>
#include <iostream>

void WebLogger::start(int port) {
    if (running_) return;
    running_ = true;
    server_thread_ = std::thread(&WebLogger::serverLoop, this, port);
    server_thread_.detach();
    std::cout << "[WebLogger] Server thread started on port " << port << std::endl;
}

void WebLogger::stop() {
    running_ = false;
    data_cv_.notify_all();
}

void WebLogger::log(const std::string& key, double value) {
    if (!running_) return;
    auto now = std::chrono::system_clock::now();
    long long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (send_queue_.size() > 10000) send_queue_.pop();
        send_queue_.push({key, value, timestamp});
    }
    data_cv_.notify_all();
}

nlohmann::json WebLogger::getBufferJSON() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    nlohmann::json j_arr = nlohmann::json::array();
    while (!send_queue_.empty()) {
        auto& d = send_queue_.front();
        j_arr.push_back({{"k", d.key}, {"v", d.value}, {"t", d.timestamp}});
        send_queue_.pop();
    }
    return j_arr;
}

WebLogger::~WebLogger() { stop(); }

void WebLogger::serverLoop(int port) {
    httplib::Server svr;

#ifdef WEB_DEBUG_STATIC_DIR
    svr.set_mount_point("/", WEB_DEBUG_STATIC_DIR);
#else
    svr.set_mount_point("/", "../web_debug/static");
#endif

    svr.Get("/health", [](const httplib::Request&, httplib::Response& res){ res.set_content("OK", "text/plain"); });

    svr.Get("/data", [&](const httplib::Request&, httplib::Response& res) {
        res.set_content_provider(
            "text/event-stream",
            [&](size_t /*offset*/, httplib::DataSink &sink) {
                while (running_) {
                    nlohmann::json j_batch;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        data_cv_.wait_for(lock, std::chrono::milliseconds(100), [&]{ return !send_queue_.empty() || !running_; });
                        if (!running_) break;
                        if (!send_queue_.empty()) {
                            j_batch = nlohmann::json::array();
                            while (!send_queue_.empty()) {
                                auto d = send_queue_.front();
                                j_batch.push_back({{"k", d.key}, {"v", d.value}, {"t", d.timestamp}});
                                send_queue_.pop();
                            }
                        }
                    }
                    if (!j_batch.empty()) {
                        std::string payload = "data: " + j_batch.dump() + "\n\n";
                        if (!sink.is_writable()) return false;
                        sink.write(payload.data(), payload.size());
                    }
                }
                return true;
            });
    });

    svr.listen("0.0.0.0", port);
}

#endif // WEB_DEBUG
