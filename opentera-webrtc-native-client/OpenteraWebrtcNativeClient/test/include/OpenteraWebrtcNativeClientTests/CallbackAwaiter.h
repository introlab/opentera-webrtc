#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H

#include <atomic>
#include <chrono>

namespace introlab
{
    class CallbackAwaiter
    {
        std::atomic_int m_count;
        const int m_maxCount;
        std::chrono::steady_clock::time_point m_begin;

    public:
        explicit CallbackAwaiter(int maxCount);

        void wait();
        bool done();
    };
}

#endif
