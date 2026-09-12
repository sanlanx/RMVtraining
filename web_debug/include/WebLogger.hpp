#pragma once

#include <string>

// 核心控制宏：如果 CMake 中没有开启 WEB_DEBUG，以下代码全部为空，不占用任何资源
#ifdef WEB_DEBUG

#include <thread>
#include <mutex>
#include <queue>
#include <atomic>
#include <condition_variable>
#include "../3rdparty/json.hpp"

class WebLogger {
public:
    static WebLogger& getInstance() {
        static WebLogger instance;
        return instance;
    }

    // 启动服务器 (非阻塞)
    void start(int port = 8080);
    
    // 停止服务器
    void stop();

    // 记录数据：key为图表名，value为数值
    void log(const std::string& key, double value);

    // 获取当前缓冲的数据 (内部使用)
    nlohmann::json getBufferJSON();
        
    std::condition_variable& getDataCondition() { return data_cv_; }
    std::mutex& getDataMutex() { return queue_mutex_; }

private:
    WebLogger() = default;
    ~WebLogger();
    WebLogger(const WebLogger&) = delete;
    WebLogger& operator=(const WebLogger&) = delete;

    void serverLoop(int port);

    std::atomic<bool> running_{false};
    std::thread server_thread_;
    
    struct LogData {
        std::string key;
        double value;
        long long timestamp;
    };
    
    std::mutex queue_mutex_;
    std::condition_variable data_cv_;
    std::queue<LogData> send_queue_;
};

// 宏接口
#define START_WEB_SERVER(port) WebLogger::getInstance().start(port)
#define WEB_LOG(key, value) WebLogger::getInstance().log(key, value)

#else

// 哑接口 (Dummy Interface)
#define START_WEB_SERVER(port) ((void)0)
#define WEB_LOG(key, value) ((void)0)

#endif // WEB_DEBUG
