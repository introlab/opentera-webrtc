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

#include <OpenteraWebrtcNativeGStreamer/Decoders/Vp9GStreamerVideoDecoders.h>
#include <OpenteraWebrtcNativeGStreamer/Utils/GStreamerSupport.h>

using namespace opentera;
using namespace std;

Vp9GStreamerVideoDecoder::Vp9GStreamerVideoDecoder(string decoderPipeline, bool resetPipelineOnSizeChanges)
    : GStreamerVideoDecoder(mediaTypeCaps(), move(decoderPipeline), resetPipelineOnSizeChanges)
{
}

const char* Vp9GStreamerVideoDecoder::mediaTypeCaps()
{
    return "video/x-vp9";
}

const char* Vp9GStreamerVideoDecoder::codecName()
{
    return cricket::kVp9CodecName;
}

SoftwareVp9GStreamerVideoDecoder::SoftwareVp9GStreamerVideoDecoder() : Vp9GStreamerVideoDecoder("vp9dec") {}

webrtc::VideoDecoder::DecoderInfo SoftwareVp9GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vp9dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool SoftwareVp9GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vp9dec");
}

bool SoftwareVp9GStreamerVideoDecoder::isHardwareAccelerated()
{
    return false;
}


VaapiVp9GStreamerVideoDecoder::VaapiVp9GStreamerVideoDecoder() : Vp9GStreamerVideoDecoder("vaapivp9dec ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiVp9GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapivp9dec";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool VaapiVp9GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vaapivp9dec") && gst::elementFactoryExists("vaapipostproc");
}

bool VaapiVp9GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}


TegraVp9GStreamerVideoDecoder::TegraVp9GStreamerVideoDecoder()
    : Vp9GStreamerVideoDecoder("nvv4l2decoder ! nvvidconv", true)
{
}

webrtc::VideoDecoder::DecoderInfo TegraVp9GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer nvv4l2decoder vp9";
    info.is_hardware_accelerated = isHardwareAccelerated();
    return info;
}

bool TegraVp9GStreamerVideoDecoder::isSupported()
{
    return gst::elementFactoryExists("vp9enc") && gst::elementFactoryExists("nvv4l2decoder") &&
           gst::elementFactoryExists("nvvidconv") &&
           gst::testEncoderDecoderPipeline("vp9enc ! nvv4l2decoder ! nvvidconv");
}

bool TegraVp9GStreamerVideoDecoder::isHardwareAccelerated()
{
    return true;
}
