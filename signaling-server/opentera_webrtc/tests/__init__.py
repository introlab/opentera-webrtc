import asyncio

from typing import Optional


class AsyncRunner:
    def __init__(self, event_loop: Optional[asyncio.BaseEventLoop] = None) -> None:
        self.loop = event_loop or asyncio.new_event_loop()

    def run_async(self, coroutine):
        return self.loop.run_until_complete(coroutine)

    def __del__(self):
        self.loop.close()
