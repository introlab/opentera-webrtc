import time

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


def on_error(error):
    # This callback is called from the internal client thread.
    print('error:')
    print('\t{}\n'.format(error))


def on_data_channel_opened(client):
    # This callback is called from the internal client thread.
    print('on_data_channel_opened:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_data_channel_closed(client):
    # This callback is called from the internal client thread.
    print('on_data_channel_closed:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))


def on_data_channel_error(client, error):
    # This callback is called from the internal client thread.
    print('on_data_channel_error:')
    print('\tid={}, name={}, data={}\n'.format(
        client.id, client.name, client.data))
    print('\terror={}\n'.format(error))


def on_data_channel_message_string(client, message):
    # This callback is called from the internal client thread.
    print('on_data_channel_message_string:')
    print('\tmessage={}\n'.format(message))


if __name__ == '__main__':
    signaling_server_configuration = webrtc.SignalingServerConfiguration.create_with_data(
        'ws://localhost:8080/signaling', 'Python', None, 'chat', 'abc')
    ice_servers = webrtc.IceServer.fetch_from_server(
        'http://localhost:8080/iceservers', 'abc')
    webrtc_configuration = webrtc.WebrtcConfiguration.create(ice_servers)
    data_channel_configuration = webrtc.DataChannelConfiguration.create()

    client = webrtc.DataChannelClient(
        signaling_server_configuration, webrtc_configuration, data_channel_configuration)

    client.on_signaling_connection_opened = on_signaling_connection_opened
    client.on_signaling_connection_closed = on_signaling_connection_closed
    client.on_signaling_connection_error = on_signaling_connection_error

    client.on_room_clients_changed = on_room_clients_changed

    client.on_client_connected = on_client_connected
    client.on_client_disconnected = on_client_disconnected
    client.on_client_connection_failed = on_client_connection_failed

    client.on_error = on_error

    client.on_data_channel_opened = on_data_channel_opened
    client.on_data_channel_closed = on_data_channel_closed
    client.on_data_channel_error = on_data_channel_error
    client.on_data_channel_message_string = on_data_channel_message_string

    client.connect()

    i = 0
    while True:
        if client.is_rtc_connected:
            i += 1
            client.send_to_all('Message {}'.format(i))
        time.sleep(10)

    client.close_sync()
