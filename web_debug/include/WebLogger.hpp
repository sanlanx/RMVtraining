#pragma once

#include <string>

// web_debug 的 CMake 目标会导出 WEB_DEBUG。未启用时，文件末尾的宏会变为空操作，
// 同时不会求值传入的参数。
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

    // 在后台线程启动 HTTP 服务。服务运行期间重复调用不会产生任何操作；
    // 默认监听本机地址。
    void start(int port = 8080, const std::string& host = "127.0.0.1");

    // 停止监听，并等待服务器线程和正在进行的 SSE 流退出。
    // 可以重复调用，之后也可以再次调用 start() 启动服务。
    void stop();

    // 向有容量上限的广播缓冲区加入一个数值样本。记录日志不会等待网页读取样本，
    // 内存占用始终受到限制。
    void log(const std::string& key, double value);

    bool isRunning() const noexcept;

    // 返回不会改变内部状态的快照，供诊断和测试使用。
    nlohmann::json getBufferJSON() const;

private:
    struct LogData {
        std::string key;
        double value;
        std::int64_t timestamp;
        std::uint64_t sequence;
    };

    // 所有客户端共享这段历史记录，但通过各自独立的序号读取。
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

// 这些宏会有意忽略传入参数。例如关闭调试时，WEB_LOG("fps", expensive())
// 不会调用 expensive()。
#define START_WEB_SERVER(port) ((void)0)
#define STOP_WEB_SERVER() ((void)0)
#define WEB_LOG(key, value) ((void)0)

#endif // WEB_DEBUG
