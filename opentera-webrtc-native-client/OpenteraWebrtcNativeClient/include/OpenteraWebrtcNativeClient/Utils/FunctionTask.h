#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H

#include <rtc_base/event.h>
#include <rtc_base/thread.h>

#include <type_traits>

namespace opentera
{
    template<class F>
    using enable_if_void_result_t = std::enable_if_t<std::is_same<void, std::invoke_result_t<F>>::value, void>;

    template<class F>
    using enable_if_not_void_result_t =
        std::enable_if_t<!std::is_same<void, std::invoke_result_t<F>>::value, std::invoke_result_t<F>>;

    template<class F>
    enable_if_void_result_t<F> callSync(rtc::Thread* thread, F&& function)
    {
        if (thread->IsCurrent())
        {
            return function();
        }
        else
        {
            rtc::Event event;
            thread->PostTask(
                [&, function = std::forward<F>(function)]()
                {
                    function();
                    event.Set();
                });
            event.Wait(rtc::Event::kForever);
        }
    }

    template<class F>
    enable_if_not_void_result_t<F> callSync(rtc::Thread* thread, F&& function)
    {
        if (thread->IsCurrent())
        {
            return function();
        }
        else
        {
            rtc::Event event;
            std::invoke_result_t<F> returnedValue;
            thread->PostTask(
                [&, function = std::forward<F>(function)]()
                {
                    returnedValue = std::move(function());
                    event.Set();
                });
            event.Wait(rtc::Event::kForever);
            return returnedValue;
        }
    }

    template<class F>
    inline void callAsync(rtc::Thread* thread, F&& function)
    {
        thread->PostTask([function = std::forward<F>(function)]() { function(); });
    }
}

#endif
