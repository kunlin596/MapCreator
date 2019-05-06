#pragma once

#include <log4cxx/logger.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#ifdef LOG4CXX_LOCATION
#undef LOG4CXX_LOCATION
#endif
#define LOG4CXX_LOCATION ::log4cxx::spi::LocationInfo(__FILENAME__, __PRETTY_FUNCTION__, __LINE__)

#define LOGGER(name) static ::log4cxx::LoggerPtr logger = ::log4cxx::Logger::getLogger(name)

#define LOG_VERBOSE(message) LOG4CXX_VERBOSE(logger, message)
#define LOG_INFO(message)    LOG4CXX_INFO(logger, message)
#define LOG_DEBUG(message)   LOG4CXX_DEBUG(logger, message)
#define LOG_WARN(message)    LOG4CXX_WARN(logger, message)
#define LOG_ERROR(message)   LOG4CXX_ERROR(logger, message)
