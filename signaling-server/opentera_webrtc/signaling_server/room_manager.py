import asyncio


class RoomManager:
    def __init__(self, web_socket_client_manager):
        self._web_socket_client_manager = web_socket_client_manager
        self._room_by_id = {}
        self._ids_by_room = {}
        self._client_name_by_id = {}
        self._client_datum_by_id = {}

        self._lock = None

    async def clear_all(self):
        self._create_lock_if_none()
        async with self._lock:
            self._room_by_id.clear()
            self._ids_by_room.clear()
            self._client_name_by_id.clear()
            self._client_datum_by_id.clear()

    async def add_client(self, id, client_name, client_data, room):
        self._create_lock_if_none()
        async with self._lock:
            if id in self._room_by_id:
                return

            self._room_by_id[id] = room

            if room in self._ids_by_room:
                if id not in self._ids_by_room[room]:
                    self._ids_by_room[room].append(id)
            else:
                self._ids_by_room[room] = [id]

            self._client_name_by_id[id] = client_name
            self._client_datum_by_id[id] = client_data

    async def remove_client(self, id):
        self._create_lock_if_none()
        async with self._lock:
            if id in self._room_by_id:
                room = self._room_by_id[id]
                del self._room_by_id[id]

                if room in self._ids_by_room:
                    self._ids_by_room[room].remove(id)
                    if len(self._ids_by_room[room]) == 0:
                        del self._ids_by_room[room]

            if id in self._client_name_by_id:
                del self._client_name_by_id[id]

            if id in self._client_datum_by_id:
                del self._client_datum_by_id[id]

    async def get_room(self, id):
        self._create_lock_if_none()
        async with self._lock:
            if id in self._room_by_id:
                return self._room_by_id[id]
            else:
                return None

    async def list_clients(self, room):
        self._create_lock_if_none()
        async with self._lock:
            if room in self._ids_by_room:
                clients = []
                for id in self._ids_by_room[room]:
                    if id in self._client_name_by_id and id in self._client_datum_by_id:
                        clients.append({
                            'id': id,
                            'name': self._client_name_by_id[id],
                            'data': self._client_datum_by_id[id]
                        })
                return clients
            else:
                return []

    async def send_to_all(self, message, room=None, skip_id=None):
        if room == None:
            await self._web_socket_client_manager.send_to_all(message)
        else:
            clients = await self.list_clients(room)
            ids = [c['id'] for c in clients if c['id'] != skip_id]
            await self._web_socket_client_manager.send_to(message, ids)

    async def get_client_name(self, id):
        self._create_lock_if_none()
        async with self._lock:
            if id in self._client_name_by_id:
                return self._client_name_by_id[id]
            else:
                return None

    def _create_lock_if_none(self):
        if self._lock is None:
            self._lock = asyncio.Lock()
