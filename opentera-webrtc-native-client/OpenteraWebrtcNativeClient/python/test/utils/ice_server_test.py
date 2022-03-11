import unittest

import opentera_webrtc.native_client as webrtc

from signaling_server_runner import SignalingServerRunner


class IceServerTestCase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        super(IceServerTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(IceServerTestCase, cls).tearDownClass()

    def test_constructor__url__should_set_the_attributes(self):
        testee = webrtc.IceServer('url1')

        self.assertEqual(testee.urls, ['url1'])
        self.assertEqual(testee.username, '')
        self.assertEqual(testee.credential, '')

    def test_constructor__url_username_credential__should_set_the_attributes(self):
        testee = webrtc.IceServer('url1', 'user', 'password')

        self.assertEqual(testee.urls, ['url1'])
        self.assertEqual(testee.username, 'user')
        self.assertEqual(testee.credential, 'password')

    def test_constructor__urls__should_set_the_attributes(self):
        testee = webrtc.IceServer(['url1', 'url2'])

        self.assertEqual(testee.urls, ['url1', 'url2'])
        self.assertEqual(testee.username, '')
        self.assertEqual(testee.credential, '')

    def test_constructor__urls__should_set_the_attributes(self):
        testee = webrtc.IceServer(['url1', 'url2'], 'user', 'password')

        self.assertEqual(testee.urls, ['url1', 'url2'])
        self.assertEqual(testee.username, 'user')
        self.assertEqual(testee.credential, 'password')

    def test_fetch_from_server__invalid_url__should(self):
        with self.assertRaises(RuntimeError):
            webrtc.IceServer.fetch_from_server('http://localhost:8080/ice', '')

    def test_fetch_from_server__wrong_password__should_return_an_empty_list(self):
        ice_servers = webrtc.IceServer.fetch_from_server('http://localhost:8080/iceservers', '')
        self.assertEqual(ice_servers, [])

    def test_fetch_from_server__right_password__should_return_the_server_ice_servers(self):
        ice_servers = webrtc.IceServer.fetch_from_server('http://localhost:8080/iceservers', 'abc')
        self.assertEqual(len(ice_servers), 1)
        self.assertEqual(ice_servers[0].urls, ['stun:stun.l.google.com:19302'])
        self.assertEqual(ice_servers[0].username, '')
        self.assertEqual(ice_servers[0].credential, '')
