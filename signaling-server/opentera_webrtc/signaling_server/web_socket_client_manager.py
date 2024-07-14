import uuid

import asyncio


class WebSocketClientManager:
    def __init__(self):
        self._ws_by_id = {}

        self._lock = None

    async def add_ws(self, ws):
        self._create_lock_if_none()
        async with self._lock:
            id = self._generate_id()
            self._ws_by_id[id] = ws
            return id

    async def close(self, id):
        self._create_lock_if_none()
        async with self._lock:
            if id in self._ws_by_id:
                await self._ws_by_id[id].close()
                del self._ws_by_id[id]

    async def close_all(self):
        self._create_lock_if_none()
        async with self._lock:
            asyncio.gather(*[ws.close() for ws in self._ws_by_id.values()])
            self._ws_by_id.clear()

    async def send_to(self, message, ids):
        if isinstance(ids, str):
            ids = [ids]

        self._create_lock_if_none()
        async with self._lock:
            tasks = []

            for id in ids:
                if id in self._ws_by_id and not self._ws_by_id[id].closed:
                    tasks.append(asyncio.create_task(self._ws_by_id[id].send_str(message)))

            if len(tasks) == 0:
                return
            await asyncio.wait(tasks)

    async def send_to_all(self, message):
        self._create_lock_if_none()
        async with self._lock:
            tasks = []

            for ws in self._ws_by_id.values():
                if not ws.closed:
                    tasks.append(asyncio.create_task(ws.send_str(message)))

            if len(tasks) == 0:
                return
            await asyncio.wait(tasks)

    async def list_ids(self):
        self._create_lock_if_none()
        async with self._lock:
            return list(self._ws_by_id.keys())

    def _generate_id(self):
        return str(uuid.uuid4())

    def _create_lock_if_none(self):
        if self._lock is None:
            self._lock = asyncio.Lock()
