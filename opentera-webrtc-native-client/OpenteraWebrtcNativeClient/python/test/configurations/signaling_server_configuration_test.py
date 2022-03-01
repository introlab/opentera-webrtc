import unittest

import opentera.webrtc.native_client as webrtc


class SignalingServerConfigurationTestCase(unittest.TestCase):
    def test_create__url_client_name_room__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', 'room')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, None)
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, '')

    def test_create__url_client_name_client_data_room__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', {'data': 10}, 'room')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, {'data': 10})
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, '')

    def test_create__url_client_name_room_password__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', room='room', password='password')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, None)
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, 'password')

    def test_create__url_client_name_client_data_room_password__should_set_the_attributes(self):
        testee = webrtc.SignalingServerConfiguration.create('url', 'name', {'data': 10}, 'room', 'password')

        self.assertEqual(testee.url, 'url')
        self.assertEqual(testee.client_name, 'name')
        self.assertEqual(testee.client_data, {'data': 10})
        self.assertEqual(testee.room, 'room')
        self.assertEqual(testee.password, 'password')
