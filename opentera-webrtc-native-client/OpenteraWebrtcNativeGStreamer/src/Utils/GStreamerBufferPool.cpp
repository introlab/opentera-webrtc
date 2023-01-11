#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerBufferPool.h>

using namespace opentera;

GStreamerBufferPool::GStreamerBufferPool() = default;

bool GStreamerBufferPool::initialize(size_t bufferSize)
{
    m_bufferPool = gst::unique_from_ptr(gst_buffer_pool_new());
    GstStructure* bufferPoolConfig = gst_buffer_pool_get_config(m_bufferPool.get());
    gst_buffer_pool_config_set_params(bufferPoolConfig, nullptr, bufferSize, 1, 0);
    return gst_buffer_pool_set_config(m_bufferPool.get(), bufferPoolConfig) &&
           gst_buffer_pool_set_active(m_bufferPool.get(), true);
}
