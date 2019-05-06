#include <log4cxx/logger.h>

#define LOGGER(name) static ::log4cxx::LoggerPtr logger = ::log4cxx::Logger::getLogger(name);

#define LOG_VERBOSE(message) LOG4CXX_VERBOSE(logger, message)
#define LOG_INFO(message)    LOG4CXX_INFO(logger, message)
#define LOG_DEBUG(message)   LOG4CXX_DEBUG(logger, message)
#define LOG_WARN(message)    LOG4CXX_WARN(logger, message)
#define LOG_ERROR(message)   LOG4CXX_ERROR(logger, message)

