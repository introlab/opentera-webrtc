import threading

import websocket

from callback_awaiter import CallbackAwaiter
from failure_test_case import FailureTestCase
from signaling_server_runner import SignalingServerRunner


class WebSocketInactiveClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(WebSocketInactiveClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(WebSocketInactiveClientTestCase, cls).tearDownClass()

    def test_socketio_inactive_is_disconnected_after_timeout(self):
        awaiter = CallbackAwaiter(2, 15)

        def on_open(ws):
            awaiter.done()

        def on_message(ws, message):
            pass

        def on_error(ws, error):
            awaiter.done()
            awaiter.done()
            self.add_failure(error)

        def on_close(ws, close_status_code, close_msg):
            awaiter.done()

        websocket.enableTrace(True)
        ws = websocket.WebSocketApp('ws://localhost:8080/signaling',
                                    on_open=on_open, on_message=on_message, on_error=on_error, on_close=on_close)

        thread = threading.Thread(target=ws.run_forever)
        thread.start()

        awaiter.wait()
        ws.close()
