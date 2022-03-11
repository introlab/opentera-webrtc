import unittest

import numpy as np
import opentera_webrtc.native_client as webrtc


class AudioSourceTestCase(unittest.TestCase):
    def test_constructor__should_only_support_valid_bits_per_sample(self):
        webrtc.AudioSource(webrtc.AudioSourceConfiguration.create(10), 8, 48000, 1)
        webrtc.AudioSource(webrtc.AudioSourceConfiguration.create(10), 16, 48000, 1)
        webrtc.AudioSource(webrtc.AudioSourceConfiguration.create(10), 32, 48000, 1)

        with self.assertRaises(RuntimeError):
            webrtc.AudioSource(webrtc.AudioSourceConfiguration.create(10), 7, 48000, 1)

    def test_send_frame__should_only_support_valid_frame(self):
        testee = webrtc.AudioSource(webrtc.AudioSourceConfiguration.create(10), 8, 48000, 2)

        with self.assertRaises(ValueError) as cm:
            testee.send_frame(np.zeros(10, dtype=np.int16))
        self.assertEqual(str(cm.exception), 'Invalid frame data type.')

        with self.assertRaises(ValueError) as cm:
            testee.send_frame(np.zeros((1, 10), dtype=np.int8))
        self.assertEqual(str(cm.exception), 'The frame must have 1 dimension.')

        with self.assertRaises(ValueError) as cm:
            testee.send_frame(np.zeros(11, dtype=np.int8))
        self.assertEqual(str(cm.exception),
                         'The frame size must be a multiple of (bytes_per_sample * number_of_channels).')

        testee.send_frame(np.zeros(10, dtype=np.int8))
