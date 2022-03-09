import unittest

import opentera_webrtc.native_client as webrtc


class DataChannelConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_the_attributes(self):
        testee = webrtc.DataChannelConfiguration.create()

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, '')

    def test_create__ordered__should_set_the_attributes(self):
        testee = webrtc.DataChannelConfiguration.create(False)

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, '')

    def test_create_protocol__should_set_the_attributes(self):
        testee = webrtc.DataChannelConfiguration.create_protocol('a')

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, 'a')

    def test_create__ordered_protocol__should_set_the_attributes(self):
        testee = webrtc.DataChannelConfiguration.create(False, 'a')

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, 'a')

    def test_create_max_packet_life_time__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_packet_life_time(10)

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, 10)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, '')

    def test_create_max_packet_life_time__ordered__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_packet_life_time(False, 10)

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, 10)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, '')

    def test_create_max_packet_life_time__protocol__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_packet_life_time(10, 'a')

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, 10)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, 'a')

    def test_create_max_packet_life_time__ordered_protocol__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_packet_life_time(False, 10, 'a')

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, 10)
        self.assertEqual(testee.max_retransmits, None)
        self.assertEqual(testee.protocol, 'a')

    def test_create_max_retransmits__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_retransmits(10)

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, 10)
        self.assertEqual(testee.protocol, '')

    def test_create_max_retransmits__ordered__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_retransmits(False, 10)

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, 10)
        self.assertEqual(testee.protocol, '')

    def test_max_retransmits__protocol__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_retransmits(10, 'a')

        self.assertEqual(testee.ordered, True)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, 10)
        self.assertEqual(testee.protocol, 'a')

    def test_create_max_retransmits__ordered_protocol__shouldSetTheAttributes(self):
        testee = webrtc.DataChannelConfiguration.create_max_retransmits(False, 10, 'a')

        self.assertEqual(testee.ordered, False)
        self.assertEqual(testee.max_packet_life_time, None)
        self.assertEqual(testee.max_retransmits, 10)
        self.assertEqual(testee.protocol, 'a')
