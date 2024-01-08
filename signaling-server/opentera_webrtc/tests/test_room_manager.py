import unittest

from signaling_server.room_manager import RoomManager
from tests import AsyncRunner


class WebSocketClientManagerMock:
    def __init__(self):
        self.messages = []

    async def add_ws(self, ws):
        raise NotImplementedError()

    async def close(self, id):
        raise NotImplementedError()

    async def send_to(self, message, ids):
        if isinstance(ids, str):
            ids = [ids]

        for id in ids:
            self.messages.append({
                'message': message,
                'to': id
            })

    async def send_to_all(self, message):
        self.messages.append({
            'message': message,
            'to': None
        })

    async def list_ids(self):
        raise NotImplementedError()

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
        self._web_socket_client_manager_mock = WebSocketClientManagerMock()
        self._room_manager = RoomManager(self._web_socket_client_manager_mock)
        self.sync_runner = AsyncRunner()

    def test_get_room(self):
        self._run_async(self._room_manager.add_client('id1', 'name1', 'data1', 'room1'))
        self._run_async(self._room_manager.add_client('id2', 'name2', 'data2', 'room2'))

        self.assertEqual(self._run_async(self._room_manager.get_room('id1')), 'room1')
        self.assertEqual(self._run_async(self._room_manager.get_room('id2')), 'room2')
        self.assertEqual(self._run_async(
            self._room_manager.get_room('id3')), None)

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

        self._run_async(self._room_manager.send_to_all('message1'))
        self._run_async(self._room_manager.send_to_all('message2', room='room1'))
        self._run_async(self._room_manager.send_to_all('message3', room='room1', skip_id='id1'))

        self.assertEqual(len(self._web_socket_client_manager_mock.messages), 4)
        self.assertTrue({'message': 'message1', 'to': None} in self._web_socket_client_manager_mock.messages)
        self.assertTrue({'message': 'message2', 'to': 'id1'} in self._web_socket_client_manager_mock.messages)
        self.assertTrue({'message': 'message2', 'to': 'id2'} in self._web_socket_client_manager_mock.messages)
        self.assertTrue({'message': 'message3', 'to': 'id2'} in self._web_socket_client_manager_mock.messages)

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
