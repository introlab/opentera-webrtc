#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H

#include <atomic>
#include <chrono>

namespace opentera
{
    class CallbackAwaiter
    {
        std::atomic_int m_count;
        const int m_maxCount;
        std::chrono::seconds m_timeout;
        std::chrono::steady_clock::time_point m_begin;

    public:
        CallbackAwaiter(int maxCount, std::chrono::seconds timeout);

        void wait(const char* file, int line);
        bool done();
    };
}

#endif
