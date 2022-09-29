#include <OpenteraWebrtcNativeClient/Utils/FunctionTask.h>

#include <OpenteraWebrtcNativeClientTests/CallbackAwaiter.h>

#include <gtest/gtest.h>

#include <thread>

using namespace opentera;
using namespace std;

class FunctionTaskTests : public ::testing::Test
{
protected:
    unique_ptr<rtc::Thread> m_thread;

    void SetUp() override
    {
        m_thread = move(rtc::Thread::Create());
        m_thread->Start();
    }

    void TearDown() override { m_thread->Stop(); }
};

TEST_F(FunctionTaskTests, callSync_int_shouldCallTheFunctionAndWaitTheResult)
{
    constexpr chrono::milliseconds SleepDuration = 500ms;

    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    int result = callSync(
        m_thread.get(),
        [this, SleepDuration]()
        {
            this_thread::sleep_for(SleepDuration);
            EXPECT_TRUE(m_thread->IsCurrent());
            return 10;
        });
    chrono::steady_clock::time_point end = chrono::steady_clock::now();

    EXPECT_EQ(result, 10);
    EXPECT_GE(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), (SleepDuration - 100ms).count());
    EXPECT_LT(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), (SleepDuration + 300ms).count());
}

TEST_F(FunctionTaskTests, callSync_intRecursive_shouldNotLock)
{
    int result = callSync(
        m_thread.get(),
        [this]()
        {
            return callSync(
                m_thread.get(),
                [this]()
                {
                    EXPECT_TRUE(m_thread->IsCurrent());
                    return 10;
                });
        });

    EXPECT_EQ(result, 10);
}

TEST_F(FunctionTaskTests, callSync_void_shouldCallTheFunctionAndWait)
{
    constexpr chrono::milliseconds SleepDuration = 500ms;

    bool flag = false;
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    callSync(
        m_thread.get(),
        [this, SleepDuration, &flag]()
        {
            this_thread::sleep_for(SleepDuration);
            flag = true;
            EXPECT_TRUE(m_thread->IsCurrent());
        });
    chrono::steady_clock::time_point end = chrono::steady_clock::now();

    EXPECT_TRUE(flag);
    EXPECT_GE(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), (SleepDuration - 100ms).count());
    EXPECT_LT(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), (SleepDuration + 300ms).count());
}

TEST_F(FunctionTaskTests, callSync_voidRecursive_shouldNotLock)
{
    bool flag = false;
    callSync(
        m_thread.get(),
        [this, &flag]()
        {
            callSync(
                m_thread.get(),
                [this, &flag]()
                {
                    flag = true;
                    EXPECT_TRUE(m_thread->IsCurrent());
                });
        });

    EXPECT_TRUE(flag);
}

TEST_F(FunctionTaskTests, callSync_void_shouldCallTheFunctionAndNotWait)
{
    constexpr chrono::milliseconds SleepDuration = 100ms;

    CallbackAwaiter awaiter(1, 1s);
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    callAsync(
        m_thread.get(),
        [this, SleepDuration, &awaiter]()
        {
            this_thread::sleep_for(SleepDuration);
            awaiter.done();
            EXPECT_TRUE(m_thread->IsCurrent());
        });
    chrono::steady_clock::time_point end = chrono::steady_clock::now();

    EXPECT_NEAR(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), 0, 10);
    awaiter.wait(__FILE__, __LINE__);
}

TEST_F(FunctionTaskTests, callSync_voidRecursive_shouldCallTheFunction)
{
    constexpr chrono::milliseconds SleepDuration = 100ms;

    CallbackAwaiter awaiter(1, 1s);
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    callAsync(
        m_thread.get(),
        [this, SleepDuration, &awaiter]()
        {
            callAsync(
                m_thread.get(),
                [this, SleepDuration, &awaiter]()
                {
                    this_thread::sleep_for(SleepDuration);
                    awaiter.done();
                    EXPECT_TRUE(m_thread->IsCurrent());
                });
        });
    chrono::steady_clock::time_point end = chrono::steady_clock::now();

    EXPECT_NEAR(chrono::duration_cast<chrono::milliseconds>(end - begin).count(), 0, 10);
    awaiter.wait(__FILE__, __LINE__);
}
