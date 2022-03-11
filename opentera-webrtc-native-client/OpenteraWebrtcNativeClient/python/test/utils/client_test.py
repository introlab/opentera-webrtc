import unittest

import opentera_webrtc.native_client as webrtc


class ClientTestCase(unittest.TestCase):
    def test_constructor__client__should_set_the_attributes(self):
        testee = webrtc.Client('id1', 'name1', {'flag': 10})

        self.assertEqual(testee.id, 'id1')
        self.assertEqual(testee.name, 'name1')
        self.assertEqual(testee.data, {'flag': 10})

    def test_constructor__room_client__should_set_the_attributes(self):
        testee = webrtc.RoomClient('id1', 'name1', {'flag': 10}, True)

        self.assertEqual(testee.id, 'id1')
        self.assertEqual(testee.name, 'name1')
        self.assertEqual(testee.data, {'flag': 10})
        self.assertEqual(testee.is_connected, True)
