import unittest

import opentera_webrtc.native_client as webrtc


class WebrtcConfigurationTestCase(unittest.TestCase):
    def test_create__should_set_the_attributes(self):
        testee = webrtc.WebrtcConfiguration.create()

        self.assertEqual(testee.ice_servers, [])

    def test_create__ice_servers__should_set_the_attributes(self):
        testee = webrtc.WebrtcConfiguration.create([webrtc.IceServer('url1')])

        self.assertEqual(len(testee.ice_servers), 1)
        self.assertEqual(testee.ice_servers[0].urls, ['url1'])
