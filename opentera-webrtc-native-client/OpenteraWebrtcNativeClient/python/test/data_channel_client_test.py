import time

import opentera_webrtc.native_client as webrtc

from callback_awaiter import CallbackAwaiter
from failure_test_case import FailureTestCase
from signaling_server_runner import SignalingServerRunner

DEFAULT_WEBRTC_CONFIGURATION = webrtc.WebrtcConfiguration.create([webrtc.IceServer('stun:stun.l.google.com:19302')])


class DisconnectedDataChannelClientTestCase(FailureTestCase):
    def setUp(self):
        super(DisconnectedDataChannelClientTestCase, self).setUp()
        self._client1 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c1', 'cd1', 'chat', ""),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

    def tearDown(self):
        self._client1.close_sync()
        super(DisconnectedDataChannelClientTestCase, self).tearDown()

    def test_is_connected__should_return_false(self):
        self.assertEqual(self._client1.is_connected, False)

    def test_is_rtc_connected__should_return_false(self):
        self.assertEqual(self._client1.is_rtc_connected, False)

    def test_id__should_return_an_empty_string(self):
        self.assertEqual(self._client1.id, '')

    def test_connected_room_client_ids__should_return_an_empty_list(self):
        self.assertEqual(self._client1.connected_room_client_ids, [])

    def test_room_clients__should_return_an_empty_list(self):
        self.assertEqual(self._client1.room_clients, [])


class WrongPasswordDataChannelClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(WrongPasswordDataChannelClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(WrongPasswordDataChannelClientTestCase, cls).tearDownClass()

    def setUp(self):
        super(WrongPasswordDataChannelClientTestCase, self).setUp()
        self._client1 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c1', 'cd1', 'chat', ''),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

        self._client1.on_error = lambda: self.add_failure('client1.on_error')

    def tearDown(self):
        self._client1.close_sync()
        super(WrongPasswordDataChannelClientTestCase, self).tearDown()

    def test_connect__should_generate_an_error(self):
        awaiter = CallbackAwaiter(2, 15)

        def on_signaling_connection_opened():
            self.add_failure('on_signaling_connection_opened')
            awaiter.done()
            awaiter.done()

        def on_signaling_connection_error(error):
            self.add_failure_assert_equal(self._client1.is_connected, False)
            self.add_failure_assert_equal(self._client1.is_rtc_connected, False)
            self.add_failure_assert_equal(self._client1.id, '')
            self.add_failure_assert_equal(self._client1.connected_room_client_ids, [])
            self.add_failure_assert_equal(self._client1.room_clients, [])
            awaiter.done()

        def on_signaling_connection_closed():
            awaiter.done()

        self._client1.on_signaling_connection_opened = on_signaling_connection_opened
        self._client1.on_signaling_connection_error = on_signaling_connection_error
        self._client1.on_signaling_connection_closed = on_signaling_connection_closed

        self._client1.connect()
        awaiter.wait()


class SingleDataChannelClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(SingleDataChannelClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(SingleDataChannelClientTestCase, cls).tearDownClass()

    def setUp(self):
        super(SingleDataChannelClientTestCase, self).setUp()
        self._client1 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c1', 'cd1', 'chat', 'abc'),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

        self._client1.on_error = lambda: self.add_failure('client1.on_error')

    def tearDown(self):
        self._client1.close_sync()
        super(SingleDataChannelClientTestCase, self).tearDown()

    def test_on_room_clients_changed__should_be_call_after_the_connection(self):
        awaiter = CallbackAwaiter(2, 15)

        def on_signaling_connection_opened():
            awaiter.done()

        def on_room_clients_changed(room_clients):
            self.add_failure_assert_equal(len(room_clients), 1)
            self.add_failure_assert_equal(room_clients[0].id, self._client1.id)
            self.add_failure_assert_equal(room_clients[0].name, 'c1')
            self.add_failure_assert_equal(room_clients[0].data, 'cd1')
            self.add_failure_assert_equal(room_clients[0].is_connected, True)
            awaiter.done()

        self._client1.on_signaling_connection_opened = on_signaling_connection_opened
        self._client1.on_room_clients_changed = on_room_clients_changed

        self._client1.connect()
        awaiter.wait()


class RightPasswordDataChannelClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(RightPasswordDataChannelClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(RightPasswordDataChannelClientTestCase, cls).tearDownClass()

    def setUp(self):
        super(RightPasswordDataChannelClientTestCase, self).setUp()
        awaiter = CallbackAwaiter(3, 15)

        def on_signaling_connection_opened():
            awaiter.done()

        self._client1 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c1', 'cd1', 'chat', 'abc'),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

        self._client2 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c2', 'cd2', 'chat', 'abc'),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

        self._client3 = webrtc.DataChannelClient(
            webrtc.SignalingServerConfiguration.create_with_data('ws://localhost:8080/signaling', 'c3', 'cd3', 'chat', 'abc'),
            DEFAULT_WEBRTC_CONFIGURATION,
            webrtc.DataChannelConfiguration.create())

        self._client1.on_signaling_connection_opened = on_signaling_connection_opened
        self._client2.on_signaling_connection_opened = on_signaling_connection_opened
        self._client3.on_signaling_connection_opened = on_signaling_connection_opened

        self._client1.on_error = lambda: self.add_failure('client1.on_error')
        self._client2.on_error = lambda: self.add_failure('client2.on_error')
        self._client3.on_error = lambda: self.add_failure('client3.on_error')

        self._client1.connect()
        time.sleep(0.25)
        self._client2.connect()
        time.sleep(0.25)
        self._client3.connect()
        awaiter.wait()

        self._clientId1 = self._client1.id
        self._clientId2 = self._client2.id
        self._clientId3 = self._client3.id

    def tearDown(self):
        self._client1.close_sync()
        self._client2.close_sync()
        self._client3.close_sync()
        super(RightPasswordDataChannelClientTestCase, self).tearDown()

    def test_is_connected__should_return_true(self):
        self.assertEqual(self._client1.is_connected, True)
        self.assertEqual(self._client2.is_connected, True)
        self.assertEqual(self._client3.is_connected, True)

    def test_is_rtc_connected__should_return_false(self):
        self.assertEqual(self._client1.is_rtc_connected, False)
        self.assertEqual(self._client2.is_rtc_connected, False)
        self.assertEqual(self._client3.is_rtc_connected, False)

    def test_id__should_not_return_an_empty_string(self):
        self.assertNotEqual(self._clientId1, '')
        self.assertNotEqual(self._clientId2, '')
        self.assertNotEqual(self._clientId3, '')

    def test_connected_room_client_ids__should_return_an_empty_list(self):
        self.assertEqual(self._client1.connected_room_client_ids, [])
        self.assertEqual(self._client2.connected_room_client_ids, [])
        self.assertEqual(self._client3.connected_room_client_ids, [])

    def test_get_room_client__should_return_the_specified_client_or_default(self):
        self.assertEqual(self._client1.get_room_client(self._clientId1).name, 'c1')
        self.assertEqual(self._client1.get_room_client(self._clientId2).name, 'c2')
        self.assertEqual(self._client1.get_room_client(self._clientId3).name, 'c3')
        self.assertEqual(self._client1.get_room_client('').name, '')

    def test_room_clients__should_return_all_clients(self):
        room_clients1 = [c.name for c in self._client1.room_clients]
        self.assertIn('c1', room_clients1)
        self.assertIn('c2', room_clients1)
        self.assertIn('c3', room_clients1)

        room_clients2 = [c.name for c in self._client1.room_clients]
        self.assertIn('c1', room_clients2)
        self.assertIn('c2', room_clients2)
        self.assertIn('c3', room_clients2)

        room_clients3 = [c.name for c in self._client1.room_clients]
        self.assertIn('c1', room_clients3)
        self.assertIn('c2', room_clients3)
        self.assertIn('c3', room_clients3)

    def test_call_all__should_call_all_clients(self):
        awaiter1 = CallbackAwaiter(2, 60)
        awaiter2 = CallbackAwaiter(2, 60)
        awaiter3 = CallbackAwaiter(2, 60)

        def on_data_channel_opened1(client):
            if awaiter1.done():
                self.add_failure_assert_true(self._client1.is_rtc_connected)
                ids = self._client1.connected_room_client_ids
                self.add_failure_assert_equal(len(ids), 2)
                self.add_failure_assert_in(self._clientId2, ids)
                self.add_failure_assert_in(self._clientId3, ids)

        def on_data_channel_opened2(client):
            if awaiter2.done():
                self.add_failure_assert_true(self._client2.is_rtc_connected)
                ids = self._client2.connected_room_client_ids
                self.add_failure_assert_equal(len(ids), 2)
                self.add_failure_assert_in(self._clientId1, ids)
                self.add_failure_assert_in(self._clientId3, ids)

        def on_data_channel_opened3(client):
            if awaiter3.done():
                self.add_failure_assert_true(self._client3.is_rtc_connected)
                ids = self._client3.connected_room_client_ids
                self.add_failure_assert_equal(len(ids), 2)
                self.add_failure_assert_in(self._clientId1, ids)
                self.add_failure_assert_in(self._clientId2, ids)

        self._client1.on_data_channel_opened = on_data_channel_opened1
        self._client2.on_data_channel_opened = on_data_channel_opened2
        self._client3.on_data_channel_opened = on_data_channel_opened3

        self._client1.call_all()
        awaiter1.wait()
        awaiter2.wait()
        awaiter3.wait()

    def test_call_ids__should_call_the_specified_client(self):
        awaiter = CallbackAwaiter(2, 60)

        def on_data_channel_opened1(client):
            self.add_failure_assert_equal(client.id, self._clientId2)
            awaiter.done()

        def on_data_channel_opened2(client):
            self.add_failure_assert_equal(client.id, self._clientId1)
            awaiter.done()

        def on_data_channel_opened3(client):
            self.add_failure('on_data_channel_opened3')

        self._client1.on_data_channel_opened = on_data_channel_opened1
        self._client2.on_data_channel_opened = on_data_channel_opened2
        self._client3.on_data_channel_opened = on_data_channel_opened3

        self._client1.call_ids([self._clientId2])
        awaiter.wait()

    def test_on_client_connected__should_be_called_after_a_call(self):
        awaiter = CallbackAwaiter(2, 60)

        def on_client_connected1(client):
            self.add_failure_assert_equal(client.id, self._clientId2)
            awaiter.done()

        def on_client_connected2(client):
            self.add_failure_assert_equal(client.id, self._clientId1)
            awaiter.done()

        def on_client_connected3(client):
            self.add_failure('on_client_connected3')

        self._client1.on_client_connected = on_client_connected1
        self._client2.on_client_connected = on_client_connected2
        self._client3.on_client_connected = on_client_connected3

        self._client1.call_ids([self._clientId2])
        awaiter.wait()

    def test_on_client_disconnected__should_be_called_after_hang_up_all_call(self):
        awaiter = CallbackAwaiter(2, 60)

        def on_data_channel_opened1(client):
            self._client1.hang_up_all()

        def on_client_disconnected1(client):
            self.add_failure_assert_equal(client.id, self._clientId2)
            awaiter.done()

        def on_client_disconnected2(client):
            self.add_failure_assert_equal(client.id, self._clientId1)
            awaiter.done()

        def on_client_disconnected3(client):
            self.add_failure('on_client_disconnected3')

        self._client1.on_data_channel_opened = on_data_channel_opened1
        self._client1.on_client_disconnected = on_client_disconnected1
        self._client2.on_client_disconnected = on_client_disconnected2
        self._client3.on_client_disconnected = on_client_disconnected3

        self._client1.call_ids([self._clientId2])
        awaiter.wait()

    def test_call_acceptor__should_be_able_to_reject_a_call_and_on_call_rejected_should_be_called(self):
        awaiter = CallbackAwaiter(4, 60)

        def on_finish():
            if awaiter.done():
                self.add_failure_assert_true(self._client1.is_rtc_connected)
                self.add_failure_assert_equal(self._client1.connected_room_client_ids, [self._clientId2])

                self.add_failure_assert_true(self._client2.is_rtc_connected)
                self.add_failure_assert_equal(self._client2.connected_room_client_ids, [self._clientId1])

                self.add_failure_assert_false(self._client3.is_rtc_connected)
                self.add_failure_assert_equal(self._client3.connected_room_client_ids, [])

        def on_client_connected1(client):
            self.add_failure_assert_equal(client.id, self._clientId2)
            on_finish()

        def on_client_connected2(client):
            self.add_failure_assert_equal(client.id, self._clientId1)
            on_finish()

        def on_client_connected3(client):
            self.add_failure('on_client_connected3')

        def call_acceptor1(client):
            self.add_failure('call_acceptor1')
            return True

        def call_acceptor2(client):
            if client.id != self._clientId1 and client.id != self._clientId3:
                self.add_failure('call_acceptor2')

            return client.id == self._clientId1

        def call_acceptor3(client):
            if client.id != self._clientId1:
                self.add_failure('call_acceptor3')

            return client.id == self._clientId2

        def on_call_rejected1(client):
            self.add_failure_assert_equal(client.id, self._clientId3)
            on_finish()

        def on_call_rejected2(client):
            self.add_failure_assert_equal(client.id, self._clientId3)
            on_finish()

        def on_call_rejected3(client):
            self.add_failure(on_call_rejected3)

        self._client1.on_client_connected = on_client_connected1
        self._client2.on_client_connected = on_client_connected2
        self._client3.on_client_connected = on_client_connected3

        self._client1.call_acceptor = call_acceptor1
        self._client2.call_acceptor = call_acceptor2
        self._client3.call_acceptor = call_acceptor3

        self._client1.on_call_rejected = on_call_rejected1
        self._client2.on_call_rejected = on_call_rejected2
        self._client3.on_call_rejected = on_call_rejected3

        self._client1.call_all()
        awaiter.wait()

    def test_hang_up_all__should_hang_up_all_clients(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        half_on_data_channel_closed_awaiter = CallbackAwaiter(4, 60)
        on_data_channel_closed_awaiter = CallbackAwaiter(6, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.hang_up_all()

        def on_data_channel_closed(client):
            if half_on_data_channel_closed_awaiter.done():
                self._client2.hang_up_all()
            on_data_channel_closed_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_closed = on_data_channel_closed
        self._client2.on_data_channel_closed = on_data_channel_closed
        self._client3.on_data_channel_closed = on_data_channel_closed

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        half_on_data_channel_closed_awaiter.wait()
        on_data_channel_closed_awaiter.wait()

    def test_close_all_room_peer_connections__should_close_all_room_peer_connections(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        on_data_channel_closed_awaiter = CallbackAwaiter(6, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.close_all_room_peer_connections()

        def on_data_channel_closed(client):
            on_data_channel_closed_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_closed = on_data_channel_closed
        self._client2.on_data_channel_closed = on_data_channel_closed
        self._client3.on_data_channel_closed = on_data_channel_closed

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        on_data_channel_closed_awaiter.wait()

    def test_send_to__binary__should_send_the_data_to_the_specified_clients(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        on_data_channel_message_awaiter = CallbackAwaiter(3, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.send_to(b'a', [self._clientId2])
                self._client2.send_to(b'b', [self._clientId3])
                self._client3.send_to(b'c', [self._clientId1])

        def on_data_channel_message_binary1(client, data):
            self.add_failure_assert_equal(client.id, self._clientId3)
            self.add_failure_assert_equal(data, b'c')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_binary2(client, data):
            self.add_failure_assert_equal(client.id, self._clientId1)
            self.add_failure_assert_equal(data, b'a')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_binary3(client, data):
            self.add_failure_assert_equal(client.id, self._clientId2)
            self.add_failure_assert_equal(data, b'b')
            on_data_channel_message_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_message_binary = on_data_channel_message_binary1
        self._client2.on_data_channel_message_binary = on_data_channel_message_binary2
        self._client3.on_data_channel_message_binary = on_data_channel_message_binary3

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        on_data_channel_message_awaiter.wait()

    def test_send_to__string__should_send_the_data_to_the_specified_clients(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        on_data_channel_message_awaiter = CallbackAwaiter(3, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.send_to('a', [self._clientId2])
                self._client2.send_to('b', [self._clientId3])
                self._client3.send_to('c', [self._clientId1])

        def on_data_channel_message_string1(client, data):
            self.add_failure_assert_equal(client.id, self._clientId3)
            self.add_failure_assert_equal(data, 'c')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_string2(client, data):
            self.add_failure_assert_equal(client.id, self._clientId1)
            self.add_failure_assert_equal(data, 'a')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_string3(client, data):
            self.add_failure_assert_equal(client.id, self._clientId2)
            self.add_failure_assert_equal(data, 'b')
            on_data_channel_message_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_message_string = on_data_channel_message_string1
        self._client2.on_data_channel_message_string = on_data_channel_message_string2
        self._client3.on_data_channel_message_string = on_data_channel_message_string3

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        on_data_channel_message_awaiter.wait()

    def test_send_to_all__binary__should_send_the_data_to_all_clients(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        on_data_channel_message_awaiter = CallbackAwaiter(6, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.send_to_all(b'a')
                self._client2.send_to_all(b'b')
                self._client3.send_to_all(b'c')

        def on_data_channel_message_binary1(client, data):
            if client.id == self._clientId2:
                self.add_failure_assert_equal(data, b'b')
            elif client.id == self._clientId3:
                self.add_failure_assert_equal(data, b'c')
            else:
                self.add_failure('on_data_channel_message_binary1')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_binary2(client, data):
            if client.id == self._clientId1:
                self.add_failure_assert_equal(data, b'a')
            elif client.id == self._clientId3:
                self.add_failure_assert_equal(data, b'c')
            else:
                self.add_failure('on_data_channel_message_binary2')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_binary3(client, data):
            if client.id == self._clientId1:
                self.add_failure_assert_equal(data, b'a')
            elif client.id == self._clientId2:
                self.add_failure_assert_equal(data, b'b')
            else:
                self.add_failure('on_data_channel_message_binary3')
            on_data_channel_message_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_message_binary = on_data_channel_message_binary1
        self._client2.on_data_channel_message_binary = on_data_channel_message_binary2
        self._client3.on_data_channel_message_binary = on_data_channel_message_binary3

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        on_data_channel_message_awaiter.wait()

    def test_send_to_all__string__should_send_the_data_to_all_clients(self):
        on_data_channel_opened_awaiter = CallbackAwaiter(6, 60)
        on_data_channel_message_awaiter = CallbackAwaiter(6, 60)

        def on_data_channel_opened(client):
            if on_data_channel_opened_awaiter.done():
                self._client1.send_to_all('a')
                self._client2.send_to_all('b')
                self._client3.send_to_all('c')

        def on_data_channel_message_string1(client, data):
            if client.id == self._clientId2:
                self.add_failure_assert_equal(data, 'b')
            elif client.id == self._clientId3:
                self.add_failure_assert_equal(data, 'c')
            else:
                self.add_failure('on_data_channel_message_binary1')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_string2(client, data):
            if client.id == self._clientId1:
                self.add_failure_assert_equal(data, 'a')
            elif client.id == self._clientId3:
                self.add_failure_assert_equal(data, 'c')
            else:
                self.add_failure('on_data_channel_message_binary2')
            on_data_channel_message_awaiter.done()

        def on_data_channel_message_string3(client, data):
            if client.id == self._clientId1:
                self.add_failure_assert_equal(data, 'a')
            elif client.id == self._clientId2:
                self.add_failure_assert_equal(data, 'b')
            else:
                self.add_failure('on_data_channel_message_binary3')
            on_data_channel_message_awaiter.done()

        self._client1.on_data_channel_opened = on_data_channel_opened
        self._client2.on_data_channel_opened = on_data_channel_opened
        self._client3.on_data_channel_opened = on_data_channel_opened

        self._client1.on_data_channel_message_string = on_data_channel_message_string1
        self._client2.on_data_channel_message_string = on_data_channel_message_string2
        self._client3.on_data_channel_message_string = on_data_channel_message_string3

        self._client1.call_all()
        on_data_channel_opened_awaiter.wait()
        on_data_channel_message_awaiter.wait()
