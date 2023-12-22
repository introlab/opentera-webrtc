import unittest

import opentera_webrtc.native_client as webrtc


class SignalingServerConfigurationTestCase(unittest.TestCase):
    def test_create__url_client_name_room__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', 'room')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, None)
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, '')

    def test_create__url_client_name_client_data_room__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', {'data': 10}, 'room')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, {'data': 10})
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, '')

    def test_create__url_client_name_room_password__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', 'room', 'password')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, None)
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, 'password')

    def test_create__url_client_name_client_data_room_password__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', {'data': 10}, 'room', 'password')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, {'data': 10})
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, 'password')

    def test_create__all_data_types__should_set_the_attributes(self):
        testee1 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', 10, 'room', 'password')
        testee2 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', 1.5, 'room', 'password')
        testee3 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', 'data', 'room', 'password')
        testee4 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', b'data', 'room', 'password')
        testee5 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', True, 'room', 'password')
        testee6 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', None, 'room', 'password')
        testee7 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', [1, 2], 'room', 'password')
        testee8 = webrtc.SignalingServerConfiguration.create_with_data('url', 'name', {'data': 10}, 'room', 'password')

        self.assertEqual(testee1.client_data, 10)
        self.assertEqual(testee2.client_data, 1.5)
        self.assertEqual(testee3.client_data, 'data')
        self.assertEqual(testee4.client_data, b'data')
        self.assertEqual(testee5.client_data, True)
        self.assertEqual(testee6.client_data, None)
        self.assertEqual(testee7.client_data, [1, 2])
        self.assertEqual(testee8.client_data, {'data': 10})
