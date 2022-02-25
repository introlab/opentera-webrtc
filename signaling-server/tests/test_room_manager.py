import asyncio
import unittest

from typing import Optional

from room_manager import RoomManager


class AsyncRunner:
    def __init__(self, event_loop: Optional[asyncio.BaseEventLoop] = None) -> None:
        self.loop = event_loop or asyncio.new_event_loop()

    def run_async(self, coroutine):
        return self.loop.run_until_complete(coroutine)

    def __del__(self):
        self.loop.close()


class SioMock:
    def __init__(self):
        self.messages = []

    async def emit(self, event, data, to=None):
        self.messages.append({
            'event': event,
            'data': data,
            'to': to
        })


class TestRoomManager(unittest.TestCase):
    def _run_async(self, coroutine):
        return self.sync_runner.run_async(coroutine)

    def setUp(self):
        self._sio_mock = SioMock()
        self._room_manager = RoomManager(self._sio_mock)
        self.sync_runner = AsyncRunner()

    def test_get_room(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        self._run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room2'))

        self.assertEqual(self._run_async(self._room_manager.get_room('id1')), 'room1')
        self.assertEqual(self._run_async(self._room_manager.get_room('id2')), 'room2')
        self.assertEqual(self._run_async(self._room_manager.get_room('id3')), None)

    def test_list_clients(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        self._run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room1'))

        clients = self._run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 2)
        self.assertEqual(clients[0], {'id': 'id1', 'name': 'name1', 'data': 'data1'})
        self.assertEqual(clients[1], {'id': 'id2', 'name': 'name2', 'data': 'data2'})

    def test_remove_client(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))

        self.assertEqual(self._run_async(self._room_manager.get_room('id1')), 'room1')

        clients = self._run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 1)
        self.assertEqual(clients[0], {'id': 'id1', 'name': 'name1', 'data': 'data1'})


        self._run_async(self._room_manager.remove_client('id1'))


        self.assertEqual(self._run_async(self._room_manager.get_room('id1')), None)

        clients = self._run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 0)

    def test_send_to_all(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        self._run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room1'))
        self._run_async(self._room_manager.add_client('id3', 'name3', 'data3', 'room2'))

        self._run_async(self._room_manager.send_to_all('event1', 'data1'))
        self._run_async(self._room_manager.send_to_all('event2', 'data2', room='room1'))
        self._run_async(self._room_manager.send_to_all('event3', 'data3', room='room1', skip_id='id1'))

        self.assertEqual(len(self._sio_mock.messages), 4)
        self.assertTrue({'event': 'event1', 'data': 'data1', 'to': None} in self._sio_mock.messages)
        self.assertTrue({'event': 'event2', 'data': 'data2', 'to': 'id1'} in self._sio_mock.messages)
        self.assertTrue({'event': 'event2', 'data': 'data2', 'to': 'id2'} in self._sio_mock.messages)
        self.assertTrue({'event': 'event3', 'data': 'data3', 'to': 'id2'} in self._sio_mock.messages)

    def test_get_client_name(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        self._run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room1'))
        self._run_async(self._room_manager.add_client('id3', 'name3', 'data3', 'room2'))

        self.assertEqual(self._run_async(self._room_manager.get_client_name('id1')), 'name1')
        self.assertEqual(self._run_async(self._room_manager.get_client_name('id2')), 'name2')
        self.assertEqual(self._run_async(self._room_manager.get_client_name('id3')), 'name3')
        self.assertEqual(self._run_async(self._room_manager.get_client_name('id4')), None)

if __name__ == '__main__':
    unittest.main()
