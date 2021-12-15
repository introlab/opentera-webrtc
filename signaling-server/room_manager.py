import asyncio


class RoomManager:
    def __init__(self, sio):
        self._sio = sio
        self._room_by_id = {}
        self._ids_by_room = {}
        self._client_name_by_id = {}
        self._client_datum_by_id = {}

        self._lock = asyncio.Lock()

    async def add_client(self, id, client_name, client_data, room):
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
        async with self._lock:
            if id in self._room_by_id:
                return self._room_by_id[id]
            else:
                return None

    async def list_clients(self, room):
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

    async def send_to_all(self, event, data=None, room=None, skip_id=None):
        if room == None:
            await self._sio.emit(event, data)
        else:
            tasks = []
            clients = await self.list_clients(room)

            for client in clients:
                if client['id'] != skip_id:
                    tasks.append(self._sio.emit(event, data, to=client['id']))

            if tasks == []:
                return
            await asyncio.wait(tasks)

    async def get_client_name(self, id):
        async with self._lock:
            if id in self._client_name_by_id:
                return self._client_name_by_id[id]
            else:
                return None
