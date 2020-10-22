import time

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


def on_data_channel_open(client):
    print('on_data_channel_open:')
    print('\tid={}, name={}, data={}\n'.format(client.id, client.name, client.data))


def on_data_channel_closed(client):
    print('on_data_channel_closed:')
    print('\tid={}, name={}, data={}\n'.format(client.id, client.name, client.data))


def on_data_channel_error(client, error):
    print('on_data_channel_error:')
    print('\tid={}, name={}, data={}\n'.format(client.id, client.name, client.data))
    print('\terror={}\n'.format(error))


def on_data_channel_message_string(client, message):
    print('on_data_channel_message_string:')
    print('\tmessage={}\n'.format(message))


if __name__ == "__main__":
    signalling_server_configuration = webrtc.SignallingServerConfiguration.create('http://localhost:8080', 'Python', None, 'chat', 'abc')
    webrtc_configuration = webrtc.WebrtcConfiguration.create()
    data_channel_configuration = webrtc.DataChannelConfiguration.create()

    client = webrtc.DataChannelClient(signalling_server_configuration, webrtc_configuration, data_channel_configuration)

    client.on_signalling_connection_open = on_signalling_connection_open
    client.on_signalling_connection_closed = on_signalling_connection_closed
    client.on_signalling_connection_error = on_signalling_connection_error

    client.on_room_clients_changed = on_room_clients_changed

    client.on_client_connected = on_client_connected
    client.on_client_disconnected = on_client_disconnected

    client.on_error = on_error

    client.on_data_channel_open = on_data_channel_open
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
