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

/*
 *  Original file(s):
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/libwebrtc/GStreamerVideoEncoderFactory.cpp.auto.html
 *  https://opensource.apple.com/source/WebCore/WebCore-7611.3.10.0.1/platform/mediastream/libwebrtc/GStreamerVideoEncoder.cpp.auto.html
 */

#include <OpenteraWebrtcNativeGStreamer/Encoders/Vp8GStreamerVideoEncoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

Vp8GStreamerVideoEncoder::Vp8GStreamerVideoEncoder(
    string encoderPipeline,
    string encoderBitratePropertyName,
    BitRateUnit bitRatePropertyUnit,
    string keyframeIntervalPropertyName)
    : GStreamerVideoEncoder(
          mediaTypeCaps(),
          move(encoderPipeline),
          move(encoderBitratePropertyName),
          bitRatePropertyUnit,
          move(keyframeIntervalPropertyName))
{
}

const char* Vp8GStreamerVideoEncoder::mediaTypeCaps()
{
    return "video/x-vp8";
}

const char* Vp8GStreamerVideoEncoder::codecName()
{
    return cricket::kVp8CodecName;
}

int Vp8GStreamerVideoEncoder::getKeyframeInterval(const webrtc::VideoCodec& codecSettings)
{
    return codecSettings.VP8().keyFrameInterval;
}

void Vp8GStreamerVideoEncoder::populateCodecSpecificInfo(
    webrtc::CodecSpecificInfo& codecSpecificInfo,
    const webrtc::EncodedImage& encodedFrame)
{
    codecSpecificInfo.codecType = webrtc::kVideoCodecVP8;
    codecSpecificInfo.codecSpecific.VP8.keyIdx = webrtc::kNoKeyIdx;
    codecSpecificInfo.codecSpecific.VP8.nonReference =
        encodedFrame._frameType != webrtc::VideoFrameType::kVideoFrameKey;
}


SoftwareVp8GStreamerVideoEncoder::SoftwareVp8GStreamerVideoEncoder(
    [[maybe_unused]] const webrtc::CodecParameterMap& parameters)
    : Vp8GStreamerVideoEncoder(
          "vp8enc name=encoder deadline=1",
          "target-bitrate",
          BitRateUnit::BitPerSec,
          "keyframe-max-dist")
{
}

webrtc::VideoEncoder::EncoderInfo SoftwareVp8GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - vp8enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool SoftwareVp8GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("vp8enc");
}

bool SoftwareVp8GStreamerVideoEncoder::isHardwareAccelerated()
{
    return false;
}

bool SoftwareVp8GStreamerVideoEncoder::areParametersSupported(
    [[maybe_unused]] const webrtc::CodecParameterMap& parameters)
{
    return true;
}


VaapiVp8GStreamerVideoEncoder::VaapiVp8GStreamerVideoEncoder(
    [[maybe_unused]] const webrtc::CodecParameterMap& parameters)
    : Vp8GStreamerVideoEncoder("vaapivp8enc name=encoder", "bitrate", BitRateUnit::KBitPerSec, "keyframe-period")
{
}

webrtc::VideoEncoder::EncoderInfo VaapiVp8GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - vaapivp8enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool VaapiVp8GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("vaapivp8enc");
}

bool VaapiVp8GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool VaapiVp8GStreamerVideoEncoder::areParametersSupported([[maybe_unused]] const webrtc::CodecParameterMap& parameters)
{
    return true;
}


TegraVp8GStreamerVideoEncoder::TegraVp8GStreamerVideoEncoder(
    [[maybe_unused]] const webrtc::CodecParameterMap& parameters)
    : Vp8GStreamerVideoEncoder(
          "nvvidconv ! nvv4l2vp8enc name=encoder",
          "bitrate",
          BitRateUnit::BitPerSec,
          "iframeinterval")
{
}

webrtc::VideoEncoder::EncoderInfo TegraVp8GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - nvv4l2vp8enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool TegraVp8GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("nvvidconv") && gst::elementFactoryExists("nvv4l2vp8enc") &&
           gst::testEncoderDecoderPipeline("nvvidconv ! nvv4l2vp8enc");
}

bool TegraVp8GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool TegraVp8GStreamerVideoEncoder::areParametersSupported([[maybe_unused]] const webrtc::CodecParameterMap& parameters)
{
    return true;
}
