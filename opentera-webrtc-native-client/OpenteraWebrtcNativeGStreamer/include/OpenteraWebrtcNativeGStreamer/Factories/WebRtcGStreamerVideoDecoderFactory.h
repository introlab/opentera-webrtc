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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_DECODER_FACOTRY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_DECODER_FACOTRY_H

#include <memory>
#include <vector>

#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_decoder.h>
#include <api/video_codecs/video_decoder_factory.h>

namespace opentera
{
    class WebRtcGStreamerVideoDecoderFactory : public webrtc::VideoDecoderFactory
    {
    public:
        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
        std::unique_ptr<webrtc::VideoDecoder> CreateVideoDecoder(const webrtc::SdpVideoFormat& format) override;
    };
}

#endif
