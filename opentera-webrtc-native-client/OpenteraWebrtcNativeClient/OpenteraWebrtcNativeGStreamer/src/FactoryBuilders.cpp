/*
 *  Copyright (C) 2022 IntRoLab
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "FactoryBuilders.h"

// TODO: IFDEF USE_GSTREAMER for headers
#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>
#include "factories/GStreamerVideoDecoderFactory.h"

namespace opentera
{
#ifdef USE_GSTREAMER
    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactory()
    {
        // TODO: Choose the right factory for the platform
        // Should return an empty unique_ptr if no hardware acceleration decoder/encoder is available for the platform
        return std::make_unique<opentera::GStreamerVideoDecoderFactory>();
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactory()
    {
        // TODO: Choose the right factory for the platform
        // Should return an empty unique_ptr if no hardware acceleration decoder/encoder is available for the platform
        return {};
    }

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactoryOrFallbackToBuiltin()
    {
        auto factory = createGStreamerVideoDecoderFactory();
        if (factory)
        {
            return factory;
        }
        else
        {
            return webrtc::CreateBuiltinVideoDecoderFactory();
        }
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactoryOrFallbackToBuiltin()
    {
        auto factory = createGStreamerVideoEncoderFactory();
        if (factory)
        {
            return factory;
        }
        else
        {
            return webrtc::CreateBuiltinVideoEncoderFactory();
        }
    }
#else
    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactory()
    {
        throw BuiltWithoutGStreamerSupport();
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactory()
    {
        throw BuiltWithoutGStreamerSupport();
    }

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerDecoderVideoFactoryOrFallbackToBuiltin()
    {
        return webrtc::CreateBuiltinVideoDecoderFactory();
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerEncoderVideoFactoryOrFallbackToBuiltin()
    {
        return webrtc::CreateBuiltinVideoDecoderFactory();
    }
#endif
}
