#ifndef CCLOG_LOG_H
#define CCLOG_LOG_H

#include <chrono>
#include <cstdarg>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

class Logger {
public:
    enum LogLevel {
        DEBUG,
        INFO,
        WARNING,
        PERROR
    };

    enum OutputType {
        TO_CONSOLE,
        TO_FILE,
        TO_BOTH
    };

    static Logger& getInstance()
    {
        static Logger instance;
        return instance;
    }

    void setOutput(OutputType type)
    {
        outputType = type;
    }

    void setLogLevel(LogLevel level)
    {
        logLevel = level;
    }

    void setLogFile(const std::string& filename)
    {
        logFileName = filename;
    }

    void setShowDebugInfo(bool show)
    {
        showDebugInfo = show;
    }

    void log(LogLevel level,
             const std::string& file,
             int line,
             const std::string& function,
             const char* format,
             ...)
    {
        if (level < logLevel) {
            return;
        }

        va_list args;
        va_start(args, format);
        std::string message = formatMessage(format, args);
        va_end(args);

        std::string output = currentDateTime() + " [" + levelToString(level) + "] " + message;

        if (showDebugInfo) {
            output += " (" + file + ":" + std::to_string(line) + " " + function + ")";
        }

        std::lock_guard<std::mutex> guard(mtx);

        if (outputType == TO_CONSOLE || outputType == TO_BOTH) {
            std::cout << colorCode(level) << output << "\033[0m" << std::endl;
        }

        if (outputType == TO_FILE || outputType == TO_BOTH) {
            std::ofstream logFile(logFileName, std::ios_base::app);
            if (logFile.is_open()) {
                logFile << output << std::endl;
            }
        }
    }

private:
    Logger() : logLevel(INFO), outputType(TO_CONSOLE), logFileName("log.txt"), showDebugInfo(false) {}
    ~Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    LogLevel logLevel;
    OutputType outputType;
    std::string logFileName;
    bool showDebugInfo;
    std::mutex mtx;

    std::string currentDateTime()
    {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::tm time_info{};
#ifdef _WIN32
        localtime_s(&time_info, &in_time_t);
#else
        localtime_r(&in_time_t, &time_info);
#endif

        std::ostringstream ss;
        ss << std::put_time(&time_info, "%Y-%m-%d %H:%M:%S")
           << "." << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }

    std::string levelToString(LogLevel level)
    {
        switch (level) {
            case DEBUG: return "DEBUG";
            case INFO: return "INFO";
            case WARNING: return "WARNING";
            case PERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }

    std::string colorCode(LogLevel level)
    {
        switch (level) {
            case DEBUG: return "\033[34m";
            case INFO: return "\033[32m";
            case WARNING: return "\033[33m";
            case PERROR: return "\033[31m";
            default: return "\033[0m";
        }
    }

    std::string formatMessage(const char* format, va_list args)
    {
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, args);
        return std::string(buffer);
    }
};

#if defined(_MSC_VER)
#define LOG_DEBUG(format, ...) Logger::getInstance().log(Logger::DEBUG, __FILE__, __LINE__, __FUNCTION__, format, __VA_ARGS__)
#define LOG_INFO(format, ...) Logger::getInstance().log(Logger::INFO, __FILE__, __LINE__, __FUNCTION__, format, __VA_ARGS__)
#define LOG_WARNING(format, ...) Logger::getInstance().log(Logger::WARNING, __FILE__, __LINE__, __FUNCTION__, format, __VA_ARGS__)
#define LOG_ERROR(format, ...) Logger::getInstance().log(Logger::PERROR, __FILE__, __LINE__, __FUNCTION__, format, __VA_ARGS__)
#else
#define LOG_DEBUG(format, ...) Logger::getInstance().log(Logger::DEBUG, __FILE__, __LINE__, __FUNCTION__, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...) Logger::getInstance().log(Logger::INFO, __FILE__, __LINE__, __FUNCTION__, format, ##__VA_ARGS__)
#define LOG_WARNING(format, ...) Logger::getInstance().log(Logger::WARNING, __FILE__, __LINE__, __FUNCTION__, format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) Logger::getInstance().log(Logger::PERROR, __FILE__, __LINE__, __FUNCTION__, format, ##__VA_ARGS__)
#endif

#endif // CCLOG_LOG_H
