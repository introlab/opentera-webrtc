#include <OpenteraWebrtcNativeClient/Codecs/VideoCodecFactories.h>

#include <api/environment/environment_factory.h>
#include <api/video_codecs/video_decoder.h>
#include <api/video_codecs/video_encoder.h>
#include <modules/video_coding/include/video_error_codes.h>
#include <media/base/media_constants.h>

#include <gtest/gtest.h>

using namespace opentera;
using namespace std;

class DummyVideoDecoder : public webrtc::VideoDecoder
{
    webrtc::SdpVideoFormat m_format;

public:
    explicit DummyVideoDecoder(webrtc::SdpVideoFormat format) : m_format(move(format)) {}
    ~DummyVideoDecoder() override = default;

    [[nodiscard]] const webrtc::SdpVideoFormat& format() const { return m_format; }

    bool Configure(const Settings& settings) override { return true; }

    int32_t Decode(const webrtc::EncodedImage& inputImage, bool missingFrames, int64_t renderTimeMs) override
    {
        return WEBRTC_VIDEO_CODEC_OK;
    }

    int32_t RegisterDecodeCompleteCallback(webrtc::DecodedImageCallback* callback) override
    {
        return WEBRTC_VIDEO_CODEC_OK;
    };

    int32_t Release() override { return WEBRTC_VIDEO_CODEC_OK; }
};

class DummyVideoEncoder : public webrtc::VideoEncoder
{
    webrtc::SdpVideoFormat m_format;

public:
    explicit DummyVideoEncoder(webrtc::SdpVideoFormat format) : m_format(move(format)) {}
    ~DummyVideoEncoder() override = default;

    [[nodiscard]] const webrtc::SdpVideoFormat& format() const { return m_format; }

    int32_t RegisterEncodeCompleteCallback(webrtc::EncodedImageCallback* callback) override
    {
        return WEBRTC_VIDEO_CODEC_OK;
    }

    int32_t Release() override { return WEBRTC_VIDEO_CODEC_OK; }

    int32_t Encode(const webrtc::VideoFrame& frame, const vector<webrtc::VideoFrameType>* frame_types) override
    {
        return WEBRTC_VIDEO_CODEC_OK;
    }

    void SetRates(const RateControlParameters& parameters) override{};

    EncoderInfo GetEncoderInfo() const override { return {}; }
};

class DummyVideoDecoderFactory : public webrtc::VideoDecoderFactory
{
public:
    DummyVideoDecoderFactory() = default;
    ~DummyVideoDecoderFactory() override = default;

    [[nodiscard]] vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override
    {
        return {
            webrtc::SdpVideoFormat(cricket::kVp8CodecName),
            webrtc::SdpVideoFormat(cricket::kVp9CodecName),
            webrtc::SdpVideoFormat(cricket::kH264CodecName),
            webrtc::SdpVideoFormat(cricket::kAv1CodecName)};
    }

    [[nodiscard]] CodecSupport
        QueryCodecSupport(const webrtc::SdpVideoFormat& format, bool referenceScaling) const override
    {
        CodecSupport codec_support;
        codec_support.is_supported = !referenceScaling;
        codec_support.is_power_efficient = true;
        return codec_support;
    }

    // Creates a VideoDecoder for the specified format.
    unique_ptr<webrtc::VideoDecoder>
        Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format) override
    {
        return make_unique<DummyVideoDecoder>(format);
    }
};

class DummyVideoEncoderFactory : public webrtc::VideoEncoderFactory
{
public:
    DummyVideoEncoderFactory() = default;
    ~DummyVideoEncoderFactory() override = default;

    [[nodiscard]] vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override
    {
        return {
            webrtc::SdpVideoFormat(cricket::kVp8CodecName),
            webrtc::SdpVideoFormat(cricket::kVp9CodecName),
            webrtc::SdpVideoFormat(cricket::kH264CodecName),
            webrtc::SdpVideoFormat(cricket::kAv1CodecName)};
    }

    [[nodiscard]] CodecSupport
        QueryCodecSupport(const webrtc::SdpVideoFormat& format, absl::optional<string> scalabilityMode) const override
    {
        CodecSupport codec_support;
        codec_support.is_supported = !scalabilityMode.has_value();
        codec_support.is_power_efficient = true;
        return codec_support;
    }

    // Creates a VideoDecoder for the specified format.
    unique_ptr<webrtc::VideoEncoder>
        Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format) override
    {
        return make_unique<DummyVideoEncoder>(format);
    }
};


TEST(VideoCodecFactoriesTests, GetSupportedFormats_emptyForcedCodecs_shouldReturnAllFormats)
{
    unordered_set<VideoStreamCodec> forcedCodecs;

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto decoderSupportedFormats = decoderFactory.GetSupportedFormats();
    auto encoderSupportedFormats = encoderFactory.GetSupportedFormats();

    ASSERT_EQ(decoderSupportedFormats.size(), 4);
    EXPECT_EQ(decoderSupportedFormats[0].name, cricket::kVp8CodecName);
    EXPECT_EQ(decoderSupportedFormats[1].name, cricket::kVp9CodecName);
    EXPECT_EQ(decoderSupportedFormats[2].name, cricket::kH264CodecName);
    EXPECT_EQ(decoderSupportedFormats[3].name, cricket::kAv1CodecName);

    ASSERT_EQ(encoderSupportedFormats.size(), 4);
    EXPECT_EQ(encoderSupportedFormats[0].name, cricket::kVp8CodecName);
    EXPECT_EQ(encoderSupportedFormats[1].name, cricket::kVp9CodecName);
    EXPECT_EQ(encoderSupportedFormats[2].name, cricket::kH264CodecName);
    EXPECT_EQ(encoderSupportedFormats[3].name, cricket::kAv1CodecName);
}

TEST(VideoCodecFactoriesTests, GetSupportedFormats_h264Vp8ForcedCodecs_shouldReturnAllFormats)
{
    unordered_set<VideoStreamCodec> forcedCodecs{VideoStreamCodec::H264, VideoStreamCodec::VP8};

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto decoderSupportedFormats = decoderFactory.GetSupportedFormats();
    auto encoderSupportedFormats = encoderFactory.GetSupportedFormats();

    ASSERT_EQ(decoderSupportedFormats.size(), 2);
    EXPECT_EQ(decoderSupportedFormats[0].name, cricket::kVp8CodecName);
    EXPECT_EQ(decoderSupportedFormats[1].name, cricket::kH264CodecName);

    ASSERT_EQ(encoderSupportedFormats.size(), 2);
    EXPECT_EQ(encoderSupportedFormats[0].name, cricket::kVp8CodecName);
    EXPECT_EQ(encoderSupportedFormats[1].name, cricket::kH264CodecName);
}

TEST(VideoCodecFactoriesTests, QueryCodecSupport_scaling_shouldReturnNotSupported)
{
    unordered_set<VideoStreamCodec> forcedCodecs;

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto decoderSupport = decoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp8CodecName), true);
    auto encoderSupport = encoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp9CodecName), "");

    EXPECT_FALSE(decoderSupport.is_supported);
    EXPECT_TRUE(decoderSupport.is_power_efficient);

    EXPECT_FALSE(encoderSupport.is_supported);
    EXPECT_TRUE(encoderSupport.is_power_efficient);
}

TEST(VideoCodecFactoriesTests, QueryCodecSupport_emptyForcedCodecs_shouldReturnSupported)
{
    unordered_set<VideoStreamCodec> forcedCodecs;

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto decoderSupport = decoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp8CodecName), false);
    auto encoderSupport =
        encoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp9CodecName), absl::nullopt);

    EXPECT_TRUE(decoderSupport.is_supported);
    EXPECT_TRUE(decoderSupport.is_power_efficient);

    EXPECT_TRUE(encoderSupport.is_supported);
    EXPECT_TRUE(encoderSupport.is_power_efficient);
}

TEST(VideoCodecFactoriesTests, QueryCodecSupport_vp8ForcedCodecs_shouldReturnSupportedIfVp8)
{
    unordered_set<VideoStreamCodec> forcedCodecs{VideoStreamCodec::VP8, VideoStreamCodec::VP9};

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto vp8DecoderSupport = decoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp8CodecName), false);
    auto av1DecoderSupport = decoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kAv1CodecName), false);

    auto vp9EncoderSupport =
        encoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kVp9CodecName), absl::nullopt);
    auto av1EncoderSupport =
        encoderFactory.QueryCodecSupport(webrtc::SdpVideoFormat(cricket::kAv1CodecName), absl::nullopt);

    EXPECT_TRUE(vp8DecoderSupport.is_supported);
    EXPECT_TRUE(vp8DecoderSupport.is_power_efficient);
    EXPECT_FALSE(av1DecoderSupport.is_supported);
    EXPECT_FALSE(av1DecoderSupport.is_power_efficient);

    EXPECT_TRUE(vp9EncoderSupport.is_supported);
    EXPECT_TRUE(vp9EncoderSupport.is_power_efficient);
    EXPECT_FALSE(av1EncoderSupport.is_supported);
    EXPECT_FALSE(av1EncoderSupport.is_power_efficient);
}

TEST(VideoCodecFactoriesTests, Create_shouldCallDummyFactory)
{
    unordered_set<VideoStreamCodec> forcedCodecs;

    ForcedCodecVideoDecoderFactory decoderFactory(make_unique<DummyVideoDecoderFactory>(), forcedCodecs);
    ForcedCodecVideoEncoderFactory encoderFactory(make_unique<DummyVideoEncoderFactory>(), forcedCodecs);

    auto environmentFactory = webrtc::EnvironmentFactory();
    auto env = environmentFactory.Create();
    auto decoder = decoderFactory.Create(env, webrtc::SdpVideoFormat(cricket::kVp9CodecName));
    auto encoder = encoderFactory.Create(env, webrtc::SdpVideoFormat(cricket::kH264CodecName));

    EXPECT_EQ(dynamic_cast<DummyVideoDecoder&>(*decoder).format().name, cricket::kVp9CodecName);
    EXPECT_EQ(dynamic_cast<DummyVideoEncoder&>(*encoder).format().name, cricket::kH264CodecName);
}
