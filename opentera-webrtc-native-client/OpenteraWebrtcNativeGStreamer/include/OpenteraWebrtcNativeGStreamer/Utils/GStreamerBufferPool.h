/*
 *  Copyright 2022 IntRoLab
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_BUFFER_POOL_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_UTILS_GSTREAMER_BUFFER_POOL_H

#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerHelpers.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/out_ptr.h>

namespace opentera
{
    class GStreamerBufferPool
    {
        gst::unique_ptr<GstBufferPool> m_bufferPool;

    public:
        GStreamerBufferPool();

        bool initialize(size_t bufferSize);
        gst::unique_ptr<GstBuffer> acquireBuffer();
    };

    inline gst::unique_ptr<GstBuffer> GStreamerBufferPool::acquireBuffer()
    {
        gst::unique_ptr<GstBuffer> buffer;
        switch (gst_buffer_pool_acquire_buffer(m_bufferPool.get(), out_ptr(buffer), nullptr))
        {
            case GST_FLOW_OK:
                return buffer;
            default:
                return nullptr;
        }
    }
}

#endif
