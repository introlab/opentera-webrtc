from callback_awaiter import CallbackAwaiter
from failure_test_case import FailureTestCase
from signaling_server_runner import SignalingServerRunner
import socketio


class SocketIOInactiveClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(SocketIOInactiveClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(SocketIOInactiveClientTestCase, cls).tearDownClass()

    def setUp(self):
        super(SocketIOInactiveClientTestCase, self).setUp()

    def tearDown(self):
        super(SocketIOInactiveClientTestCase, self).tearDown()

    def test_socketio_inactive_is_disconnected_after_timeout(self):
        awaiter = CallbackAwaiter(2, 15)
        sio = socketio.Client()

        @sio.event
        def connect():
            awaiter.done()

        @sio.event
        def connect_error():
            awaiter.done()
            awaiter.done()
            self.add_failure('sio.connect_error')

        @sio.event
        def disconnect():
            awaiter.done()

        sio.connect('http://localhost:8080')
        awaiter.wait()
