#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_TESTS_CALLBACK_AWAITER_H

#include <atomic>

namespace introlab
{
    class CallbackAwaiter
    {
        std::atomic_int m_count;
        const int m_maxCount;

    public:
        explicit CallbackAwaiter(int maxCount);

        void wait();
        void done();
    };
}

#endif
