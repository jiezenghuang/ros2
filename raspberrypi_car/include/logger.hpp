#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <stdexcept>

template<typename ... Args>
std::string string_format( const char* format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format, args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ) { throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format, args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

class Logger
{
public:
    static Logger& get_logger()
    {
        static Logger logger;
        return logger;
    }

    void debug(const char* format, ...)
    {
        va_list args;
        write_log(Debug, string_format(format, args));
    }

    void info(const char* format, ...)
    {
        va_list args;
        write_log(Debug, string_format(format, args));
    }

    void error(const char* format, ...)
    {
        va_list args;
        write_log(Debug, string_format(format, args));
    }

private:
    Logger() {}
    ~Logger() {}

    enum LogType
    {
        Debug,
        Info,
        Error
    };

    void write_log(int type, std::string log)
    {
        std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::cout << std::put_time(std::localtime(&now), "[%F %T]");
        switch(type)
        {
            case Debug:
                std::cout << "[Debug]";
            case Info:
                std::cout << "[Info]";
            case Error:
            default:
                std::cout << "[Error]";
        }
        std::cout << log << std::endl;
    }
};
