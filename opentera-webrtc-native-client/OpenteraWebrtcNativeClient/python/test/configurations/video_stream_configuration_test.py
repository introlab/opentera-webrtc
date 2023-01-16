import unittest

import opentera_webrtc.native_client as webrtc


class VideoStreamConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_the_attributes(self):
        testee = webrtc.VideoStreamConfiguration.create()

        self.assertEqual(testee.forced_codecs, set())
        self.assertEqual(testee.force_gstreamer_hardware_acceleration, False)
        self.assertEqual(testee.use_gstreamer_software_encoder_decoder, False)

    def test_create__forced_codecs__should_set_the_attributes(self):
        testee = webrtc.VideoStreamConfiguration.create({webrtc.VideoStreamCodec.VP8})

        self.assertEqual(testee.forced_codecs, {webrtc.VideoStreamCodec.VP8})
        self.assertEqual(testee.force_gstreamer_hardware_acceleration, False)
        self.assertEqual(testee.use_gstreamer_software_encoder_decoder, False)

    def test_create__all__should_set_the_attributes(self):
        testee = webrtc.VideoStreamConfiguration.create({webrtc.VideoStreamCodec.VP9, webrtc.VideoStreamCodec.H264},
                                                        True, False)

        self.assertEqual(testee.forced_codecs, {webrtc.VideoStreamCodec.VP9, webrtc.VideoStreamCodec.H264})
        self.assertEqual(testee.force_gstreamer_hardware_acceleration, True)
        self.assertEqual(testee.use_gstreamer_software_encoder_decoder, False)
