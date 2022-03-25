#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H

#include <rtc_base/event.h>
#include <rtc_base/thread.h>

#include <type_traits>

namespace opentera
{
    template<class F>
    using enable_if_void_result_t = std::enable_if_t<std::is_same<void, std::result_of_t<F()>>::value>;

    template<class F>
    using enable_if_not_void_result_t =
        std::enable_if_t<!std::is_same<void, std::result_of_t<F()>>::value, std::result_of_t<F()>>;

    template<class F>
    enable_if_void_result_t<F> callSync(rtc::Thread* thread, const F& function)
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
    enable_if_not_void_result_t<F> callSync(rtc::Thread* thread, const F& function)
    {
        if (thread->IsCurrent())
        {
            return function();
        }
        else
        {
            rtc::Event event;
            std::result_of_t<F()> returnedValue;
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
