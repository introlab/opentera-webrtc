#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H

#include <rtc_base/event.h>
#include <rtc_base/thread.h>

#include <functional>

namespace opentera
{
    template<class T>
    class FunctionTask
    {
    public:
        FunctionTask() = delete;

        static T callSync(rtc::Thread* thread, const std::function<T()>& function)
        {
            if (thread->IsCurrent())
            {
                return function();
            }
            else
            {
                rtc::Event event;
                T returnedValue;
                thread->PostTask(
                    RTC_FROM_HERE,
                    [&]()
                    {
                        returnedValue = std::move(function());
                        event.Set();
                    });
                event.Wait(rtc::Event::kForever);
                return std::move(returnedValue);
            }
        }
    };

    template<>
    class FunctionTask<void>
    {
    public:
        FunctionTask() = delete;

        static void callSync(rtc::Thread* thread, const std::function<void()>& function)
        {
            if (thread->IsCurrent())
            {
                function();
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

        static void callAsync(rtc::Thread* thread, const std::function<void()>& function)
        {
            thread->PostTask(RTC_FROM_HERE, [=]() { function(); });
        }
    };
}

#endif
