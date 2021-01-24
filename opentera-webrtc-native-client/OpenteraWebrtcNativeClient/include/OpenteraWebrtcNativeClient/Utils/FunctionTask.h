#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_UTILS_QUEUED_TASK_UTILS_H

#include <api/proxy.h>
#include <api/task_queue/queued_task.h>
#include <rtc_base/thread.h>

#include <functional>

namespace opentera
{
    template<class T>
    class FunctionTask : public webrtc::QueuedTask
    {
        std::function<T()> m_function;
        rtc::Event m_event;
        T m_returnedValue;

    public:
        FunctionTask(const std::function<T()>& function) : m_function(function)
        {
        }

        ~FunctionTask() override = default;

        bool Run() override
        {
            m_returnedValue = std::move(m_function());
            m_event.Set();
            return false;
        }

        static T callSync(rtc::Thread* thread, const std::function<T()>& function)
        {
            if (thread->IsCurrent())
            {
                return function();
            }
            else
            {
                FunctionTask task(function);
                thread->PostTask(std::unique_ptr<QueuedTask>(&task));
                task.m_event.Wait(rtc::Event::kForever);
                return std::move(task.m_returnedValue);
            }
        }
    };

    template<>
    class FunctionTask<void> : public webrtc::QueuedTask
    {
        std::function<void()> m_function;
        bool m_isAsync;
        rtc::Event m_event;

    public:
        FunctionTask(const std::function<void()>& function, bool isAsync) : m_function(function), m_isAsync(isAsync)
        {
        }

        ~FunctionTask() override = default;

        bool Run() override
        {
            m_function();
            // Copy flag so it does not read the flag on a deleted object.
            bool isAsync = m_isAsync;
            m_event.Set();
            return isAsync;
        }

        static void callSync(rtc::Thread* thread, const std::function<void()>& function)
        {
            if (thread->IsCurrent())
            {
                return function();
            }
            else
            {
                FunctionTask task(function, false);
                thread->PostTask(std::unique_ptr<QueuedTask>(&task));
                task.m_event.Wait(rtc::Event::kForever);
            }
        }

        static void callAsync(rtc::Thread* thread, const std::function<void()>& function)
        {
            thread->PostTask(std::make_unique<FunctionTask>(function, true));
        }
    };
}

#endif
