import asyncio
import unittest

from room_manager import RoomManager

def _run_async(coroutine):
    return asyncio.get_event_loop().run_until_complete(coroutine)

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
    def setUp(self):
        self._sio_mock = SioMock()
        self._room_manager = RoomManager(self._sio_mock)

    def test_get_room(self):
        _run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        _run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room2'))

        self.assertEqual(_run_async(self._room_manager.get_room('id1')), 'room1')
        self.assertEqual(_run_async(self._room_manager.get_room('id2')), 'room2')
        self.assertEqual(_run_async(self._room_manager.get_room('id3')), None)

    def test_list_clients(self):
        _run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        _run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room1'))

        clients = _run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 2)
        self.assertEqual(clients[0], {'id': 'id1', 'name': 'name1', 'data': 'data1'})
        self.assertEqual(clients[1], {'id': 'id2', 'name': 'name2', 'data': 'data2'})

    def test_remove_client(self):
        _run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))

        self.assertEqual(_run_async(self._room_manager.get_room('id1')), 'room1')

        clients = _run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 1)
        self.assertEqual(clients[0], {'id': 'id1', 'name': 'name1', 'data': 'data1'})


        _run_async(self._room_manager.remove_client('id1'))


        self.assertEqual(_run_async(self._room_manager.get_room('id1')), None)

        clients = _run_async(self._room_manager.list_clients('room1'))
        self.assertEqual(len(clients), 0)

    def test_send_to_all(self):
        _run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        _run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room1'))
        _run_async(self._room_manager.add_client('id3', 'name3', 'data3', 'room2'))

        _run_async(self._room_manager.send_to_all('event1', 'data1'))
        _run_async(self._room_manager.send_to_all('event2', 'data2', room='room1'))
        _run_async(self._room_manager.send_to_all('event3', 'data3', room='room1', skip_id='id1'))

        self.assertEqual(len(self._sio_mock.messages), 4)
        self.assertTrue({'event': 'event1', 'data': 'data1', 'to': None} in self._sio_mock.messages)
        self.assertTrue({'event': 'event2', 'data': 'data2', 'to': 'id1'} in self._sio_mock.messages)
        self.assertTrue({'event': 'event2', 'data': 'data2', 'to': 'id2'} in self._sio_mock.messages)
        self.assertTrue({'event': 'event3', 'data': 'data3', 'to': 'id2'} in self._sio_mock.messages)


if __name__ == '__main__':
    unittest.main()
