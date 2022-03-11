import unittest

import opentera_webrtc.native_client as webrtc


class VideoSourceConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_the_attributes(self):
        testee = webrtc.VideoSourceConfiguration.create(True, False)

        self.assertEqual(testee.needs_denoising, True)
        self.assertEqual(testee.is_screencast, False)
