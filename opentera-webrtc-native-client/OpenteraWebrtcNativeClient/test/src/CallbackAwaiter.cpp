#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace introlab;
using namespace std;

CallbackAwaiter::CallbackAwaiter(int maxCount) : m_count(0), m_maxCount(maxCount)
{
}

void CallbackAwaiter::wait()
{
    constexpr int64_t Timeout = 15;

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();

    while (m_count.load() < m_maxCount)
    {
        this_thread::sleep_for(50ms);

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(end - begin).count() > Timeout)
        {
            ADD_FAILURE();
            return;
        }
    }
}

void CallbackAwaiter::done()
{
    m_count++;
}
