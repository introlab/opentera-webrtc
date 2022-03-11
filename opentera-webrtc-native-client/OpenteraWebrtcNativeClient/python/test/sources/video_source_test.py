import unittest

import numpy as np
import opentera_webrtc.native_client as webrtc


class VideoSourceTestCase(unittest.TestCase):

    def test_send_frame__should_only_support_valid_frame(self):
        testee = webrtc.VideoSource(webrtc.VideoSourceConfiguration.create(False, False))

        with self.assertRaises(ValueError) as cm:
            testee.send_frame(np.zeros((10, 10), dtype=np.int8), 0)
        self.assertEqual(str(cm.exception), 'The image must have 3 dimensions.')

        with self.assertRaises(ValueError) as cm:
            testee.send_frame(np.zeros((10, 10, 4), dtype=np.int8), 1000)
        self.assertEqual(str(cm.exception), 'The channel count must be 3.')

        testee.send_frame(np.zeros((10, 10, 3), dtype=np.int8), 2000)
