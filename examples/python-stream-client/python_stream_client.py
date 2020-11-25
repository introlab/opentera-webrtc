import os
import time

import numpy as np
import cv2

import opentera_webrtc_native_client as webrtc


def on_signalling_connection_open():
    print('on_signalling_connection_open')


def on_signalling_connection_closed():
    print('on_signalling_connection_closed')


def on_signalling_connection_error(error):
    print('on_signalling_connection_error:')
    print('\terror={}\n'.format(error))


def on_room_clients_changed(room_clients):
    print('on_room_clients_changed:')
    for c in room_clients:
        print('\tid={}, name={}, data={}, is_connected={}'.format(c.id, c.name, c.data, c.is_connected))
    print('\n')


def on_client_connected(client):
    print('on_client_connected:')
    print('\tid={}, name={}, data={}\n'.format(client.id, client.name, client.data))


def on_client_disconnected(client):
    print('on_client_disconnected:')
    print('\tid={}, name={}, data={}\n'.format(client.id, client.name, client.data))


def on_error(error):
    print('error or warning:')
    print('\t{}\n'.format(error))


if __name__ == '__main__':
    frame = cv2.imread(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'frame.png'))

    signalling_server_configuration = webrtc.SignallingServerConfiguration.create('http://localhost:8080', 'Python', None, 'chat', 'abc')
    webrtc_configuration = webrtc.WebrtcConfiguration.create()
    data_channel_configuration = webrtc.DataChannelConfiguration.create()

    video_source = webrtc.VideoSource(False, False)
    client = webrtc.StreamClient(signalling_server_configuration, webrtc_configuration, video_source)

    client.on_signalling_connection_open = on_signalling_connection_open
    client.on_signalling_connection_closed = on_signalling_connection_closed
    client.on_signalling_connection_error = on_signalling_connection_error

    client.on_room_clients_changed = on_room_clients_changed

    client.on_client_connected = on_client_connected
    client.on_client_disconnected = on_client_disconnected

    client.on_error = on_error

    client.connect()


    start_time = time.time()
    while True:
        frame_with_noise = frame + np.random.randint(0, 10, size=frame.shape, dtype=frame.dtype)
        timestamp_us = int((time.time() - start_time) * 1e6)
        video_source.send_frame(frame_with_noise, timestamp_us)

        time.sleep(0.1)

    client.close_sync()