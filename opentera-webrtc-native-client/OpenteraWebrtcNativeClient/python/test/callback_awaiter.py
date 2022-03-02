import threading
import time


class CallbackAwaiter:
    def __init__(self, max_count, timeout_seconds):
        self._count = 0
        self._max_count = max_count
        self._timeout_seconds = timeout_seconds
        self._lock = threading.Lock()

    def wait(self):
        def get_count():
            with self._lock:
                return self._count

        start_time = time.time()
        while get_count() < self._max_count:
            time.sleep(0.05)

            if time.time() - start_time > self._timeout_seconds:
                raise TimeoutError()

    def done(self):
        with self._lock:
            self._count += 1
            return self._count >= self._max_count

    def is_finished(self):
        with self._lock:
            return self._count >= self._max_count
