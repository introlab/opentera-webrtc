#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_THREAD_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_THREAD_H

#include <thread>

#if defined(WIN32) || defined(_WIN32)

enum class ThreadPriority : int
{
    Normal = 0x00000020,
    RealTime = 0x00000100,
};

#elif defined(UNIX) || defined(__unix__) || defined(LINUX) || defined(__linux__) || defined(__APPLE__)

enum class ThreadPriority : int
{
    Normal,
    RealTime,
};

#else

#error "The OS is not supported."

#endif

bool setThreadPriority(std::thread& thread, ThreadPriority priority);

#endif
