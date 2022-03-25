#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H

#include <rtc_base/event.h>
#include <rtc_base/thread.h>

#include <type_traits>

namespace opentera
{
    template<class F>
    void callSync(
        rtc::Thread* thread,
        const F& function,
        typename std::enable_if<std::is_same<void, typename std::result_of<F()>::type>::value>::type* = nullptr)
    {
        if (thread->IsCurrent())
        {
            return function();
        }
        else
        {
            rtc::Event event;
            thread->PostTask(
                RTC_FROM_HERE,
                [&]()
                {
                    function();
                    event.Set();
                });
            event.Wait(rtc::Event::kForever);
        }
    }

    template<class F>
    typename std::result_of<F()>::type callSync(
        rtc::Thread* thread,
        const F& function,
        typename std::enable_if<!std::is_same<void, typename std::result_of<F()>::type>::value>::type* = nullptr)
    {
        if (thread->IsCurrent())
        {
            return function();
        }
        else
        {
            rtc::Event event;
            typename std::result_of<F()>::type returnedValue;
            thread->PostTask(
                RTC_FROM_HERE,
                [&]()
                {
                    returnedValue = std::move(function());
                    event.Set();
                });
            event.Wait(rtc::Event::kForever);
            return returnedValue;
        }
    }

    template<class F>
    inline void callAsync(rtc::Thread* thread, const F& function)
    {
        thread->PostTask(RTC_FROM_HERE, [=]() { function(); });
    }
}

#endif
