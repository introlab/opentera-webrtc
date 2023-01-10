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

#include <OpenteraWebrtcNativeGStreamer/Decoders/Vp8GStreamerVideoDecoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

Vp8GStreamerVideoDecoder::Vp8GStreamerVideoDecoder(string decoderPipeline, bool resetPipelineOnSizeChanges)
    : GStreamerVideoDecoder(mediaTypeCaps(), move(decoderPipeline), resetPipelineOnSizeChanges)
{
}

const char* Vp8GStreamerVideoDecoder::mediaTypeCaps()
{
    return "video/x-vp8";
}

const char* Vp8GStreamerVideoDecoder::codecName()
{
    return cricket::kVp8CodecName;
}


SoftwareVp8GStreamerVideoDecoder::SoftwareVp8GStreamerVideoDecoder() : Vp8GStreamerVideoDecoder("vp8dec") {}

webrtc::VideoDecoder::DecoderInfo SoftwareVp8GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vp8dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool SoftwareVp8GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vp8dec");
}

bool SoftwareVp8GStreamerVideoDecoder::isHardwareAccelerated()
{
    return false;
}


VaapiVp8GStreamerVideoDecoder::VaapiVp8GStreamerVideoDecoder() : Vp8GStreamerVideoDecoder("vaapivp8dec ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiVp8GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapivp8dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool VaapiVp8GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vaapivp8dec") && gst::elementFactoryExists("vaapipostproc");
}

bool VaapiVp8GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}


TegraVp8GStreamerVideoDecoder::TegraVp8GStreamerVideoDecoder()
    : Vp8GStreamerVideoDecoder("nvv4l2decoder ! nvvidconv", true)
{
}

webrtc::VideoDecoder::DecoderInfo TegraVp8GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer nvv4l2decoder vp8";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool TegraVp8GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vp8enc") && gst::elementFactoryExists("nvv4l2decoder") &&
           gst::elementFactoryExists("nvvidconv") &&
           gst::testEncoderDecoderPipeline("vp8enc ! nvv4l2decoder ! nvvidconv");
}

bool TegraVp8GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}
