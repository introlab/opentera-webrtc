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

#include <OpenteraWebrtcNativeGStreamer/Decoders/H264GStreamerVideoDecoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

H264GStreamerVideoDecoder::H264GStreamerVideoDecoder(string decoderPipeline, bool resetPipelineOnSizeChanges)
    : GStreamerVideoDecoder(mediaTypeCaps(), move(decoderPipeline), resetPipelineOnSizeChanges)
{
}

const char* H264GStreamerVideoDecoder::mediaTypeCaps()
{
    return "video/x-h264";
}

const char* H264GStreamerVideoDecoder::codecName()
{
    return cricket::kH264CodecName;
}


SoftwareH264GStreamerVideoDecoder::SoftwareH264GStreamerVideoDecoder()
    : H264GStreamerVideoDecoder("h264parse ! avdec_h264")
{
}

webrtc::VideoDecoder::DecoderInfo SoftwareH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer avdec_h264";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool SoftwareH264GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("h264parse") && gst::elementFactoryExists("avdec_h264");
}

bool SoftwareH264GStreamerVideoDecoder::isHardwareAccelerated()
{
    return false;
}


VaapiH264GStreamerVideoDecoder::VaapiH264GStreamerVideoDecoder()
    : H264GStreamerVideoDecoder("vaapih264dec ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapih264dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool VaapiH264GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vaapih264dec") && gst::elementFactoryExists("vaapipostproc");
}

bool VaapiH264GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}


TegraH264GStreamerVideoDecoder::TegraH264GStreamerVideoDecoder()
    : H264GStreamerVideoDecoder("nvv4l2decoder ! nvvidconv", true)
{
}

webrtc::VideoDecoder::DecoderInfo TegraH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer nvv4l2decoder h264";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool TegraH264GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("x264enc") && gst::elementFactoryExists("nvv4l2decoder") &&
           gst::elementFactoryExists("nvvidconv") &&
           gst::testEncoderDecoderPipeline("x264enc ! nvv4l2decoder ! nvvidconv");
}

bool TegraH264GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}


V4l2H264GStreamerVideoDecoder::V4l2H264GStreamerVideoDecoder() : H264GStreamerVideoDecoder("h264parse ! v4l2h264dec") {}

webrtc::VideoDecoder::DecoderInfo V4l2H264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer v4l2h264dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool V4l2H264GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("h264parse") && gst::elementFactoryExists("v4l2h264dec");
}

bool V4l2H264GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}


AppleMediaH264GStreamerVideoDecoder::AppleMediaH264GStreamerVideoDecoder()
    : H264GStreamerVideoDecoder("h264parse ! vtdec ! videoconvert")
{
}

webrtc::VideoDecoder::DecoderInfo AppleMediaH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vtdec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool AppleMediaH264GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("h264parse") && gst::elementFactoryExists("vtdec") &&
           gst::elementFactoryExists("videoconvert");
}

bool AppleMediaH264GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}
