#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

using namespace introlab;
using namespace std;

CallbackAwaiter::CallbackAwaiter(int maxCount, chrono::seconds timeout) :
        m_count(0), m_maxCount(maxCount), m_timeout(timeout), m_begin(chrono::steady_clock::now())
{
}

void CallbackAwaiter::wait()
{
    while (m_count.load() < m_maxCount)
    {
        this_thread::sleep_for(50ms);

        chrono::steady_clock::time_point end = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::seconds>(end - m_begin) > m_timeout)
        {
            ADD_FAILURE();
            return;
        }
    }
}

bool CallbackAwaiter::done()
{
    return (++m_count) == m_count;
}
