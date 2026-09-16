#pragma once

#include <string>

// The web_debug CMake target exports WEB_DEBUG. Without it, the macros at the
// end of this file become no-ops and do not evaluate their arguments.
#ifdef WEB_DEBUG

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include "json.hpp"

namespace httplib {
class Server;
}

class WebLogger {
public:
    static WebLogger& getInstance();

    // Starts HTTP service on a background thread. Repeated calls while it is
    // running are no-ops. The default address is local-only.
    void start(int port = 8080, const std::string& host = "127.0.0.1");

    // Stops listening and joins the server thread and active SSE streams.
    // It is safe to call repeatedly, and start() may be called again later.
    void stop();

    // Adds one numeric sample to a bounded broadcast buffer. Logging never
    // waits for a page to read the sample and memory usage remains bounded.
    void log(const std::string& key, double value);

    bool isRunning() const noexcept;

    // Returns a non-destructive snapshot for diagnostics and tests.
    nlohmann::json getBufferJSON() const;

private:
    struct LogData {
        std::string key;
        double value;
        std::int64_t timestamp;
        std::uint64_t sequence;
    };

    // Clients share this history but read it through independent sequences.
    static constexpr std::size_t kBufferCapacity = 10000;

    WebLogger();
    ~WebLogger();
    WebLogger(const WebLogger&) = delete;
    WebLogger& operator=(const WebLogger&) = delete;

    void configureServer(httplib::Server& server);
    void serverLoop();

    std::atomic<bool> running_{false};
    mutable std::mutex lifecycle_mutex_;
    std::thread server_thread_;
    std::unique_ptr<httplib::Server> server_;

    mutable std::mutex buffer_mutex_;
    std::condition_variable data_cv_;
    std::deque<LogData> buffer_;
    std::uint64_t next_sequence_{1};
};

#define START_WEB_SERVER(port) WebLogger::getInstance().start(port)
#define STOP_WEB_SERVER() WebLogger::getInstance().stop()
#define WEB_LOG(key, value) WebLogger::getInstance().log((key), (value))

#else

// These macros intentionally omit their arguments. For example,
// WEB_LOG("fps", expensive()) will not call expensive() when debugging is off.
#define START_WEB_SERVER(port) ((void)0)
#define STOP_WEB_SERVER() ((void)0)
#define WEB_LOG(key, value) ((void)0)

#endif // WEB_DEBUG
