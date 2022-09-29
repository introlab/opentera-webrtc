import unittest

import opentera_webrtc.native_client as webrtc


class AudioSourceConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_none(self):
        testee = webrtc.AudioSourceConfiguration.create(10)

        self.assertEqual(testee.sound_card_total_delay_ms, 10)
        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.transient_suppression, None)

    def test_create__echo_cancellation_auto_gain_control__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(10, True, False, None, None, None, None)

        self.assertEqual(testee.sound_card_total_delay_ms, 10)
        self.assertEqual(testee.echo_cancellation, True)
        self.assertEqual(testee.auto_gain_control, False)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.transient_suppression, None)

    def test_create__noise_suppression_highpass_filter__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(10, None, None, True, False, None, None)

        self.assertEqual(testee.sound_card_total_delay_ms, 10)
        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, True)
        self.assertEqual(testee.highpass_filter, False)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.transient_suppression, None)

    def test_create__stereo_swapping__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(10, None, None, None, None, True, None)

        self.assertEqual(testee.sound_card_total_delay_ms, 10)
        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, True)
        self.assertEqual(testee.transient_suppression, None)

    def test_create__transient_suppression__should_set_the_attributes(self):
        testee = webrtc.AudioSourceConfiguration.create(10, None, None, None, None, None, False)

        self.assertEqual(testee.sound_card_total_delay_ms, 10)
        self.assertEqual(testee.echo_cancellation, None)
        self.assertEqual(testee.auto_gain_control, None)
        self.assertEqual(testee.noise_suppression, None)
        self.assertEqual(testee.highpass_filter, None)
        self.assertEqual(testee.stereo_swapping, None)
        self.assertEqual(testee.transient_suppression, False)
