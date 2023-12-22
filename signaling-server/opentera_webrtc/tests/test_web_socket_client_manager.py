import unittest

from signaling_server.web_socket_client_manager import WebSocketClientManager
from tests import AsyncRunner


class WsMock:
    def __init__(self):
        self._closed = False
        self.messages = []

    async def send_str(self, message):
        self.messages.append(message)

    async def close(self):
        self._closed = True

    @property
    def closed(self):
        return self._closed


class TestRoomManager(unittest.TestCase):
    def _run_async(self, coroutine):
        return self.sync_runner.run_async(coroutine)

    def setUp(self):
        self._ws_manager = WebSocketClientManager()
        self.sync_runner = AsyncRunner()

    def test_add_ws(self):
        id1 = self._run_async(self._ws_manager.add_ws(WsMock()))
        id2 = self._run_async(self._ws_manager.add_ws(WsMock()))

        self.assertNotEqual(id1, id2)

        ids = self._run_async(self._ws_manager.list_ids())
        self.assertEqual(len(ids), 2)
        self.assertIn(id1, ids)
        self.assertIn(id2, ids)

    def test_close(self):
        ws = WsMock()
        id = self._run_async(self._ws_manager.add_ws(ws))
        self.assertIn(id, self._run_async(self._ws_manager.list_ids()))
        self.assertFalse(ws.closed)

        self._run_async(self._ws_manager.close(id))
        self.assertNotIn(id, self._run_async(self._ws_manager.list_ids()))
        self.assertTrue(ws.closed)

    def test_send_to_send_to_all(self):
        ws1 = WsMock()
        ws2 = WsMock()
        ws3 = WsMock()
        ws4 = WsMock()
        self._run_async(ws4.close())

        id1 = self._run_async(self._ws_manager.add_ws(ws1))
        id2 = self._run_async(self._ws_manager.add_ws(ws2))
        id3 = self._run_async(self._ws_manager.add_ws(ws3))
        id4 = self._run_async(self._ws_manager.add_ws(ws4))

        self._run_async(self._ws_manager.send_to('message1', [id1, id2]))
        self._run_async(self._ws_manager.send_to('message2', id3))
        self._run_async(self._ws_manager.send_to('message3', id4))
        self._run_async(self._ws_manager.send_to_all('message4'))

        self.assertEqual(ws1.messages, ['message1', 'message4'])
        self.assertEqual(ws2.messages, ['message1', 'message4'])
        self.assertEqual(ws3.messages, ['message2', 'message4'])
        self.assertEqual(ws4.messages, [])
