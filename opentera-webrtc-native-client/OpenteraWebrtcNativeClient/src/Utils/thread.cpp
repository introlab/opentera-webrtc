#include <OpenteraWebrtcNativeClient/Utils/thread.h>

#if defined(WIN32) || defined(_WIN32)
#include <windows.h>
#elif defined(UNIX) || defined(__unix__) || defined(LINUX) || defined(__linux__)
#include <pthread.h>
#endif

using namespace std;

bool setThreadPriority(thread& thread, ThreadPriority priority)
{
#if defined(WIN32) || defined(_WIN32)
    return SetPriorityClass(thread.native_handle(), static_cast<DWORD>(priority)) != 0;
#elif defined(UNIX) || defined(__unix__) || defined(LINUX) || defined(__linux__) || defined(__APPLE__)
    int topPriority = sched_get_priority_max(SCHED_RR) - 1;
    int lowPriority = sched_get_priority_min(SCHED_RR) + 1;

    sched_param sch_params;
    switch (priority)
    {
        case ThreadPriority::Normal:
            sch_params.sched_priority = (topPriority - lowPriority - 1) / 2;
            break;
        case ThreadPriority::RealTime:
            sch_params.sched_priority = topPriority;
            break;
    }

    return pthread_setschedparam(thread.native_handle(), SCHED_RR, &sch_params) == 0;
#endif
}
