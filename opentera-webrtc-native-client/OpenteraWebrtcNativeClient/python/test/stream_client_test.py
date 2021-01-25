import time

import numpy as np
import opentera_webrtc_native_client as webrtc

from callback_awaiter import CallbackAwaiter
from failure_test_case import FailureTestCase
from signaling_server_runner import SignalingServerRunner


class StreamClientTestCase(FailureTestCase):
    @classmethod
    def setUpClass(cls):
        super(StreamClientTestCase, cls).setUpClass()
        cls._signaling_server_runner = SignalingServerRunner()

    @classmethod
    def tearDownClass(cls):
        cls._signaling_server_runner.close()
        super(StreamClientTestCase, cls).tearDownClass()

    def test_video_stream__should_be_sent_and_received(self):
        # Initialize the clients
        setup_awaiter = CallbackAwaiter(2, 15)

        frame1 = np.zeros((480, 640, 3), dtype=np.int8)
        frame1[:, :, 2] = 255
        frame2 = np.zeros((480, 640, 3), dtype=np.int8)
        frame2[:, :, 0] = 255

        video_source1 = webrtc.VideoSource(webrtc.VideoSourceConfiguration.create(False, True))
        video_source2 = webrtc.VideoSource(webrtc.VideoSourceConfiguration.create(False, True))

        def on_signaling_connection_opened():
            setup_awaiter.done()

        client1 = webrtc.StreamClient(
            webrtc.SignalingServerConfiguration.create('http://localhost:8080', 'c1', 'cd1', 'chat', 'abc'),
            webrtc.WebrtcConfiguration.create(),
            video_source1)

        client2 = webrtc.StreamClient(
            webrtc.SignalingServerConfiguration.create('http://localhost:8080', 'c2', 'cd2', 'chat', 'abc'),
            webrtc.WebrtcConfiguration.create(),
            video_source2)

        client1.on_signaling_connection_opened = on_signaling_connection_opened
        client2.on_signaling_connection_opened = on_signaling_connection_opened

        client1.on_error = lambda: self.add_failure('client1.on_error')
        client2.on_error = lambda: self.add_failure('client2.on_error')

        client1.connect()
        client2.connect()
        setup_awaiter.wait()

        # Setup the callback
        on_video_frame_awaiter1 = CallbackAwaiter(1, 15)
        on_video_frame_awaiter2 = CallbackAwaiter(1, 15)

        self._on_add_remote_stream_client_name1 = None
        self._on_add_remote_stream_client_name2 = None
        self._mean_color_1 = np.array([0, 0, 0], dtype=np.int8)
        self._mean_color_2 = np.array([0, 0, 0], dtype=np.int8)

        def on_add_remote_stream1(client):
            self._on_add_remote_stream_client_name1 = client.name

        def on_add_remote_stream2(client):
            self._on_add_remote_stream_client_name2 = client.name

        def on_remove_remote_stream(client):
            self.add_failure('on_remove_remote_stream')

        def on_video_frame_received1(client, frame, timestamp):
            self._mean_color_1 = np.mean(frame, axis=(0, 1))
            on_video_frame_awaiter1.done()

        def on_video_frame_received2(client, frame, timestamp):
            self._mean_color_2 = np.mean(frame, axis=(0, 1))
            on_video_frame_awaiter2.done()

        def on_audio_frame_received(client, data, sample_rate, number_of_channels, number_of_frames):
            self.add_failure('on_audio_frame_received')

        client1.on_add_remote_stream = on_add_remote_stream1
        client2.on_add_remote_stream = on_add_remote_stream2

        client1.on_remove_remote_stream = on_remove_remote_stream
        client2.on_remove_remote_stream = on_remove_remote_stream

        client1.on_video_frame_received = on_video_frame_received1
        client2.on_video_frame_received = on_video_frame_received2

        client1.on_audio_frame_received = on_audio_frame_received
        client2.on_audio_frame_received = on_audio_frame_received

        # Setup the call
        client1.call_all()

        start_time = time.time()
        while not on_video_frame_awaiter1.is_finished() or not on_video_frame_awaiter2.is_finished():
            timestamp_us = int((time.time() - start_time) * 1e6)
            video_source1.send_frame(frame1, timestamp_us)
            video_source2.send_frame(frame2, timestamp_us)
            time.sleep(0.005)

        client1.close_all_room_peer_connections()
        time.sleep(1)

        client1.close_sync()
        client2.close_sync()

        # Asserts
        self.assertEqual(self._on_add_remote_stream_client_name1, 'c2')
        self.assertEqual(self._on_add_remote_stream_client_name2, 'c1')

        self.assertAlmostEqual(self._mean_color_1[0], 255, delta=15)
        self.assertAlmostEqual(self._mean_color_1[1], 0, delta=15)
        self.assertAlmostEqual(self._mean_color_1[2], 0, delta=15)

        self.assertAlmostEqual(self._mean_color_2[0], 0, delta=15)
        self.assertAlmostEqual(self._mean_color_2[1], 0, delta=15)
        self.assertAlmostEqual(self._mean_color_2[2], 255, delta=15)
