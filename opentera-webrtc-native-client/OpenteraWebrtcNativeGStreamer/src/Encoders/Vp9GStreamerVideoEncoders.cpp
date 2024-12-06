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

#include <OpenteraWebrtcNativeGStreamer/Encoders/Vp9GStreamerVideoEncoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

// TODO fix VP9

Vp9GStreamerVideoEncoder::Vp9GStreamerVideoEncoder(
    const webrtc::CodecParameterMap& parameters,
    string encoderPipeline,
    string encoderBitratePropertyName,
    BitRateUnit bitRatePropertyUnit,
    string keyframeIntervalPropertyName)
    : GStreamerVideoEncoder(
          mediaTypeCaps(parameters),
          move(encoderPipeline),
          move(encoderBitratePropertyName),
          bitRatePropertyUnit,
          move(keyframeIntervalPropertyName))
{
}

string Vp9GStreamerVideoEncoder::mediaTypeCaps(const webrtc::CodecParameterMap& parameters)
{
    auto it = parameters.find(cricket::kVP9ProfileId);
    if (it == parameters.end())
    {
        return "video/x-vp9,profile=(string)0";
    }
    else
    {
        return "video/x-vp9,profile=(string)" + it->second;
    }
}

const char* Vp9GStreamerVideoEncoder::codecName()
{
    return cricket::kVp9CodecName;
}

bool Vp9GStreamerVideoEncoder::isProfile0123OrNone(const webrtc::CodecParameterMap& parameters)
{
    if (parameters.size() > 1)
    {
        return false;
    }

    auto it = parameters.find(cricket::kVP9ProfileId);
    if (it == parameters.end())
    {
        return true;
    }
    else
    {
        return it->second == "0" || it->second == "1" || it->second == "2" || it->second == "3";
    }
}

int Vp9GStreamerVideoEncoder::getKeyframeInterval(const webrtc::VideoCodec& codecSettings)
{
    return codecSettings.VP9().keyFrameInterval;
}

void Vp9GStreamerVideoEncoder::populateCodecSpecificInfo(
    webrtc::CodecSpecificInfo& codecSpecificInfo,
    [[maybe_unused]] const webrtc::EncodedImage& encodedFrame)
{
    codecSpecificInfo.codecType = webrtc::kVideoCodecVP9;

    codecSpecificInfo.codecSpecific.VP9.first_frame_in_picture = true;
    codecSpecificInfo.codecSpecific.VP9.flexible_mode = m_isFlexibleMode;

    codecSpecificInfo.codecSpecific.VP9.num_spatial_layers = 1;

    // TODO add missing
}
int Vp9GStreamerVideoEncoder::InitEncode(
    const webrtc::VideoCodec* codecSettings,
    const webrtc::VideoEncoder::Settings& settings)
{
    m_isFlexibleMode = codecSettings->VP9().flexibleMode;
    return GStreamerVideoEncoder::InitEncode(codecSettings, settings);
}


SoftwareVp9GStreamerVideoEncoder::SoftwareVp9GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : Vp9GStreamerVideoEncoder(
          parameters,
          "vp9enc name=encoder deadline=1",
          "target-bitrate",
          BitRateUnit::BitPerSec,
          "keyframe-max-dist")
{
}

webrtc::VideoEncoder::EncoderInfo SoftwareVp9GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - vp9enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool SoftwareVp9GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("vp9enc");
}

bool SoftwareVp9GStreamerVideoEncoder::isHardwareAccelerated()
{
    return false;
}

bool SoftwareVp9GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfile0123OrNone(parameters);
}


TegraVp9GStreamerVideoEncoder::TegraVp9GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters)
    : Vp9GStreamerVideoEncoder(
          parameters,
          "nvvidconv ! nvv4l2vp9enc name=encoder",
          "bitrate",
          BitRateUnit::BitPerSec,
          "iframeinterval")
{
}

webrtc::VideoEncoder::EncoderInfo TegraVp9GStreamerVideoEncoder::GetEncoderInfo() const
{
    webrtc::VideoEncoder::EncoderInfo info(GStreamerVideoEncoder::GetEncoderInfo());
    info.implementation_name = "GStreamer - nvv4l2vp9enc";
    info.is_hardware_accelerated = false;

    return info;
}

bool TegraVp9GStreamerVideoEncoder::isSupported()
{
    return gst::elementFactoryExists("nvvidconv") && gst::elementFactoryExists("nvv4l2vp9enc") &&
           gst::testEncoderDecoderPipeline("nvvidconv ! nvv4l2vp9enc");
}

bool TegraVp9GStreamerVideoEncoder::isHardwareAccelerated()
{
    return true;
}

bool TegraVp9GStreamerVideoEncoder::areParametersSupported(const webrtc::CodecParameterMap& parameters)
{
    return isProfile0123OrNone(parameters);
}
