import unittest

import opentera_webrtc_native_client as webrtc


class AudioSourceConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_none(self):
        testee = webrtc.AudioSourceConfiguration.create()

        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.typing_detection, None)
        self.assertEqual(testee.residual_echo_detector, None)

    def test_create__echo_cancellation_auto_gain_control__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(True, False, None, None, None, None, None)

        self.assertEqual(testee.echo_cancellation, True)
        self.assertEqual(testee.auto_gain_control, False)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.typing_detection, None)
        self.assertEqual(testee.residual_echo_detector, None)

    def test_create__noise_suppression_highpass_filter__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(None, None, True, False, None, None, None)

        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, True)
        self.assertEqual(testee.highpass_filter, False)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.typing_detection, None)
        self.assertEqual(testee.residual_echo_detector, None)

    def test_create__stereo_swapping_typing_detection__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(None, None, None, None, True, False, None)

        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, True)
        self.assertEqual(testee.typing_detection, False)
        self.assertEqual(testee.residual_echo_detector, None)

    def test_create__residual_echo_detector__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(None, None, None, None, None, None, True)

        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.typing_detection, None)
        self.assertEqual(testee.residual_echo_detector, True)
