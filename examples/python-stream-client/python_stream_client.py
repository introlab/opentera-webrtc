import os
import time

import numpy as np
import cv2

import opentera_webrtc.native_client as webrtc


def on_signaling_connection_opened():
    # This callback is called from the internal client thread.
    print('on_signaling_connection_opened')


def on_signaling_connection_closed():
    # This callback is called from the internal client thread.
    print('on_signaling_connection_closed')


def on_signaling_connection_error(error):
    # This callback is called from the internal client thread.
    print('on_signaling_connection_error:')
    print('\terror={}\n'.format(error))


def on_room_clients_changed(room_clients):
    # This callback is called from the internal client thread.
    print('on_room_clients_changed:')
    for c in room_clients:
        print('\tid={}, name={}, data={}, is_connected={}'.format(
            c.id, c.name, c.data, c.is_connected))
    print('\n')


def on_client_connected(client):
    # This callback is called from the internal client thread.
    print('on_client_connected:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_client_disconnected(client):
    # This callback is called from the internal client thread.
    print('on_client_disconnected:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_client_connection_failed(client):
    # This callback is called from the internal client thread.
    print('on_client_connection_failed:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_add_remote_stream(client):
    # This callback is called from the internal client thread.
    print('on_add_remote_stream:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_remove_remote_stream(client):
    # This callback is called from the internal client thread.
    print('on_remove_remote_stream:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_video_frame_received(client, image, timestampUs):
    # This callback is called from a WebRTC processing thread.
    cv2.imshow(client.id, image)
    cv2.waitKey(1)


def on_audio_frame_received(client, data, sample_rate, number_of_channels, number_of_frames):
    # This callback is called from a WebRTC processing thread.
    print('on_audio_frame_received:')
    print('\tid={}, name={}, data={}'.format(
        client.id, client.name, client.data))
    print('\tdtype={}, sample_rate={}, number_of_channels={}, number_of_frames={}'.format(
        data.dtype, sample_rate, number_of_channels, number_of_frames))


def on_mixed_audio_frame_received(data, sample_rate, number_of_channels, number_of_frames):
    # This callback is called from the audio device module thread.
    print('on_mixed_audio_frame_received:')
    print('\tdtype={}, sample_rate={}, number_of_channels={}, number_of_frames={}'.format(
        data.dtype, sample_rate, number_of_channels, number_of_frames))


def on_error(error):
    # This callback is called from the internal client thread.
    print('error:')
    print('\t{}\n'.format(error))


if __name__ == '__main__':
    frame = cv2.imread(os.path.join(os.path.dirname(
        os.path.realpath(__file__)), 'frame.png'))

    signaling_server_configuration = webrtc.SignalingServerConfiguration.create_with_data(
        'ws://localhost:8080/signaling', 'Python', None, 'chat', 'abc')
    ice_servers = webrtc.IceServer.fetch_from_server(
        'http://localhost:8080/iceservers', 'abc')
    webrtc_configuration = webrtc.WebrtcConfiguration.create(ice_servers)
    video_stream_configuration = webrtc.VideoStreamConfiguration.create()

    video_source = webrtc.VideoSource(
        webrtc.VideoSourceConfiguration.create(False, False))
    sound_card_total_delay_ms = 0
    fs = 48000
    audio_source = webrtc.AudioSource(
        webrtc.AudioSourceConfiguration.create(sound_card_total_delay_ms), 16, fs, 1)
    client = webrtc.StreamClient(
        signaling_server_configuration, webrtc_configuration, video_stream_configuration, video_source, audio_source)

    client.on_signaling_connection_opened = on_signaling_connection_opened
    client.on_signaling_connection_closed = on_signaling_connection_closed
    client.on_signaling_connection_error = on_signaling_connection_error

    client.on_room_clients_changed = on_room_clients_changed

    client.on_client_connected = on_client_connected
    client.on_client_disconnected = on_client_disconnected
    client.on_client_connection_failed = on_client_connection_failed

    client.on_add_remote_stream = on_add_remote_stream
    client.on_remove_remote_stream = on_remove_remote_stream
    client.on_video_frame_received = on_video_frame_received
    client.on_audio_frame_received = on_audio_frame_received
    client.on_mixed_audio_frame_received = on_mixed_audio_frame_received

    client.on_error = on_error

    client.connect()

    t = np.linspace(-np.pi, np.pi, fs // 200)
    audio_frame = (15000 * np.sin(t)).astype(np.int16)

    start_time = time.time()
    while True:
        frame_with_noise = frame + \
            np.random.randint(0, 10, size=frame.shape, dtype=frame.dtype)
        timestamp_us = int((time.time() - start_time) * 1e6)
        video_source.send_frame(frame_with_noise, timestamp_us)
        audio_source.send_frame(audio_frame)

        time.sleep(0.005)

    client.close_sync()
