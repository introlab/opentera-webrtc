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

#include <OpenteraWebrtcNativeGStreamer/Encoders/H264GStreamerVideoEncoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

constexpr const char* BaselineProfileLevelIdPrefix = "42";
constexpr const char* MainProfileLevelIdPrefix = "4f";
constexpr const char* High444ProfileLevelIdPrefix = "f4";

H264GStreamerVideoEncoder::H264GStreamerVideoEncoder(
    const webrtc::CodecParameterMap& parameters,
    string encoderPipeline,
    string encoderBitratePropertyName,
    BitRateUnit bitRatePropertyUnit,
    string keyframeIntervalPropertyName,
    const char* additionalMediaTypeCaps)
    : GStreamerVideoEncoder(
          mediaTypeCaps(parameters) + additionalMediaTypeCaps,
          move(encoderPipeline),
          move(encoderBitratePropertyName),
          bitRatePropertyUnit,
          move(keyframeIntervalPropertyName))
{
    auto packetizationModeIt = parameters.find(cricket::kH264FmtpPacketizationMode);
    if (packetizationModeIt == parameters.end() || packetizationModeIt->second == "0")
    {
        m_packetizationMode = webrtc::H264PacketizationMode::SingleNalUnit;
    }
    else
    {
        m_packetizationMode = webrtc::H264PacketizationMode::NonInterleaved;
    }
}

string H264GStreamerVideoEncoder::mediaTypeCaps(const webrtc::CodecParameterMap& parameters)
{
    auto it = parameters.find(cricket::kH264FmtpProfileLevelId);
    if (it == parameters.end() || it->second.find(BaselineProfileLevelIdPrefix) == 0)
    {
        return "video/x-h264,alignment=au,stream-format=byte-stream,profile=baseline";
    }
    else if (it->second.find(MainProfileLevelIdPrefix) == 0)
    {
        return "video/x-h264,alignment=au,stream-format=byte-stream,profile=main";
    }
    else if (it->second.find(High444ProfileLevelIdPrefix) == 0)
    {
        return "video/x-h264,alignment=au,stream-format=byte-stream,profile=high-4:4:4";
    }
    else
    {
        return "video/x-h264,alignment=au,stream-format=byte-stream";
    }
}

const char* H264GStreamerVideoEncoder::codecName()
{
    return cricket::kH264CodecName;
}

bool H264GStreamerVideoEncoder::isProfileBaselineOrMain(const webrtc::CodecParameterMap& parameters)
{
    auto it = parameters.find(cricket::kH264FmtpProfileLevelId);
    if (it == parameters.end())
    {
        return true;
    }
    else
    {
        return it->second.find(BaselineProfileLevelIdPrefix) == 0 || it->second.find(MainProfileLevelIdPrefix) == 0;
    }
}

bool H264GStreamerVideoEncoder::isProfileBaselineOrMainOrHigh444(const webrtc::CodecParameterMap& parameters)
{
    auto it = parameters.find(cricket::kH264FmtpProfileLevelId);
    if (it == parameters.end())
    {
        return true;
    }
    else
    {
        return it->second.find(BaselineProfileLevelIdPrefix) == 0 || it->second.find(MainProfileLevelIdPrefix) == 0 ||
               it->second.find(High444ProfileLevelIdPrefix) == 0;
    }
}

int H264GStreamerVideoEncoder::getKeyframeInterval(const webrtc::VideoCodec& codecSettings)
{
    return codecSettings.H264().keyFrameInterval;
}

void H264GStreamerVideoEncoder::populateCodecSpecificInfo(
    webrtc::CodecSpecificInfo& codecSpecificInfo,
    [[maybe_unused]] const webrtc::EncodedImage& encodedFrame)
{
    codecSpecificInfo.codecType = webrtc::kVideoCodecH264;
    codecSpecificInfo.codecSpecific.H264.packetization_mode = m_packetizationMode;
}


SoftwareH264GStreamerVideoEncoder::SoftwareH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : H264GStreamerVideoEncoder(
          parameters,
          "x264enc name=encoder tune=zerolatency ! h264parse",
          "bitrate",
          BitRateUnit::KBitPerSec,
          "key-int-max")
{
}

webrtc::VideoEncoder::EncoderInfo SoftwareH264GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - x264enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool SoftwareH264GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("x264enc") && gst::elementFactoryExists("h264parse");
}

bool SoftwareH264GStreamerVideoEncoder::isHardwareAccelerated()
{
    return false;
}

bool SoftwareH264GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfileBaselineOrMainOrHigh444(parameters);
}


VaapiH264GStreamerVideoEncoder::VaapiH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : H264GStreamerVideoEncoder(
          parameters,
          "vaapih264enc name=encoder ! h264parse",
          "bitrate",
          BitRateUnit::KBitPerSec,
          "keyframe-period")
{
}

webrtc::VideoEncoder::EncoderInfo VaapiH264GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - vaapih264enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool VaapiH264GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("vaapih264enc") && gst::elementFactoryExists("h264parse");
}

bool VaapiH264GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool VaapiH264GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfileBaselineOrMain(parameters);
}


TegraH264GStreamerVideoEncoder::TegraH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : H264GStreamerVideoEncoder(
          parameters,
          "nvvidconv ! nvv4l2h264enc name=encoder profile=" + profileFromParameters(parameters) + " ! h264parse",
          "bitrate",
          BitRateUnit::BitPerSec,
          "iframeinterval")
{
}

webrtc::VideoEncoder::EncoderInfo TegraH264GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - nvv4l2h264enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool TegraH264GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("nvvidconv") && gst::elementFactoryExists("nvv4l2h264enc") &&
           gst::testEncoderDecoderPipeline("nvvidconv ! nvv4l2h264enc");
}

bool TegraH264GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool TegraH264GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfileBaselineOrMain(parameters);
}

string TegraH264GStreamerVideoEncoder::profileFromParameters(const webrtc::CodecParameterMap& parameters)
{
    auto it = parameters.find(cricket::kH264FmtpProfileLevelId);
    if (it != parameters.end() && it->second.find(MainProfileLevelIdPrefix) == 0)
    {
        return "2";
    }
    else
    {
        return "0";
    }
}


V4l2H264GStreamerVideoEncoder::V4l2H264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : H264GStreamerVideoEncoder(
          parameters,
          "v4l2h264enc name=encoder ! h264parse",
          "extra-controls.video_bitrate",
          BitRateUnit::BitPerSec,
          "extra-controls.video_gop_size",
          ",level=(string)4.1")
{
}

webrtc::VideoEncoder::EncoderInfo V4l2H264GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - v4l2h264enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool V4l2H264GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("v4l2h264enc") && gst::elementFactoryExists("h264parse");
}

bool V4l2H264GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool V4l2H264GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfileBaselineOrMain(parameters);
}


AppleMediaH264GStreamerVideoEncoder::AppleMediaH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : H264GStreamerVideoEncoder(
          parameters,
          "vtenc_h264 name=encoder ! h264parse",
          "bitrate",
          BitRateUnit::KBitPerSec,
          "max-keyframe-interval")
{
}

webrtc::VideoEncoder::EncoderInfo AppleMediaH264GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - vtenc_h264";
    info.is_hardware_accelerated = false;

    return info;
}

bool AppleMediaH264GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("vtenc_h264") && gst::elementFactoryExists("h264parse");
}

bool AppleMediaH264GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool AppleMediaH264GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfileBaselineOrMain(parameters);
}
