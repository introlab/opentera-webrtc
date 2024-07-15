#! /usr/bin/env python3

import argparse
import asyncio
import itertools
import json
import sys
import logging

from pathlib import Path
from typing import Union

from aiohttp import web, WSMsgType
from aiohttp_index import IndexMiddleware

import ssl

from opentera_webrtc.signaling_server.room_manager import RoomManager
from opentera_webrtc.signaling_server.web_socket_client_manager import WebSocketClientManager

PROTOCOL_VERSION = 2
DISCONNECT_DELAY_S = 1
INACTIVE_DELAY_S = 5
PING_INTERVAL_S = 10


@web.middleware
async def cors_middleware(request, handler):
    response = await handler(request)
    response.headers['Cross-Origin-Opener-Policy'] = 'same-origin'
    response.headers['Cross-Origin-Embedder-Policy'] = 'require-corp'
    response.headers['Cross-Origin-Resource-Policy'] = 'cross-origin'
    return response


logging.basicConfig(format='[%(asctime)s] -- %(message)s')
logger = logging.getLogger('signaling_server')

app = web.Application(middlewares=[IndexMiddleware(), cors_middleware])

web_socket_client_manager = WebSocketClientManager()
room_manager = RoomManager(web_socket_client_manager)

password = None
ice_servers = []


async def disconnect_delayed(id):
    await asyncio.sleep(DISCONNECT_DELAY_S)
    await web_socket_client_manager.close(id)


async def disconnect_inactive_user(id):
    await asyncio.sleep(INACTIVE_DELAY_S)
    # Verify if id joined a room
    room = await room_manager.get_room(id)
    if room is None:
        logger.info('inactive: %s', id)
        await web_socket_client_manager.close(id)


async def web_socket_ping_task(ws):
    while not ws.closed:
        await ws.ping()
        await asyncio.sleep(PING_INTERVAL_S)


def event_to_message(event, data=None):
    if data is None:
        message = {'event': event}
    else:
        message = {'event': event, 'data': data}

    return json.dumps(message)


async def web_socket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    id = await web_socket_client_manager.add_ws(ws)
    logger.info('connect %s', id)
    asyncio.create_task(disconnect_inactive_user(id))
    asyncio.create_task(web_socket_ping_task(ws))

    async for msg in ws:
        if msg.type == WSMsgType.TEXT:
            try:
                data = json.loads(msg.data)
                await handle_web_socket_message(id, data)
            except Exception as e:
                logger.error('Message error (%s): %s', id, str(e))

    logger.info('disconnect %s (%s)', id, await room_manager.get_client_name(id))
    await web_socket_client_manager.close(id)
    room = await room_manager.get_room(id)
    await room_manager.remove_client(id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        await room_manager.send_to_all(event_to_message('room-clients', clients), room=room)

    return ws


async def handle_web_socket_message(id, data):
    if 'event' not in data:
        logger.error('Invalid message (%s): %s', id, str(data))
        return

    event = data['event']
    if 'data' not in data:
        data = {}
    else:
        data = data['data']

    if event == 'join-room':
        await handle_join_room(id, data)
    elif event == 'send-ice-candidate':
        await handle_ice_candidate(id, data)
    elif event == 'call-peer':
        await handle_call_peer(id, data)
    elif event == 'make-peer-call-answer':
        await handle_make_call_answer(id, data)
    elif event == 'call-all':
        await handle_call_all(id)
    elif event == 'call-ids':
        await handle_call_ids(id, data)
    elif event == 'close-all-room-peer-connections':
        await handle_close_all_peer_connections(id)
    else:
        logger.error('Not handled message (%s): %s: %s', id, event, str(data))


async def handle_join_room(id, data):
    logger.info('join_room %s (%s)', id, data)

    if not _isAuthorized(data['password'] if 'password' in data else ''):
        asyncio.create_task(disconnect_delayed(id))
        await web_socket_client_manager.send_to(event_to_message('join-room-answer', ''), id)
        return
    if (data['protocolVersion'] if 'protocolVersion' in data else 0) != PROTOCOL_VERSION:
        asyncio.create_task(disconnect_delayed(id))
        await web_socket_client_manager.send_to(event_to_message('join-room-answer', ''), id)
        return

    if 'data' not in data:
        data['data'] = {}

    await web_socket_client_manager.send_to(event_to_message('join-room-answer', id), id)

    await room_manager.add_client(id, data['name'], data['data'], data['room'])
    clients = await room_manager.list_clients(data['room'])
    await room_manager.send_to_all(event_to_message('room-clients', clients), room=data['room'])


async def handle_ice_candidate(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('send-ice-candidate %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await web_socket_client_manager.send_to(event_to_message('ice-candidate-received', data), data['toId'])


async def handle_call_peer(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('call %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await web_socket_client_manager.send_to(event_to_message('peer-call-received', data), data['toId'])


async def handle_make_call_answer(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('make-peer-call-answer %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await web_socket_client_manager.send_to(event_to_message('peer-call-answer-received', data), data['toId'])


async def handle_call_all(from_id):
    logger.info('call-all %s (%s)', from_id, await room_manager.get_client_name(from_id))
    room = await room_manager.get_room(from_id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        ids = [client['id'] for client in clients]
        await _make_peer_calls(ids)


async def handle_call_ids(from_id, ids):
    names = [(id, await room_manager.get_client_name(id)) for id in ids]
    logger.info('call-ids %s (%s) %s', from_id, await room_manager.get_client_name(from_id), names)
    room = await room_manager.get_room(from_id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        all_ids = [client['id'] for client in clients]
        ids.append(from_id)
        await _make_peer_calls(list(set(ids) & set(all_ids)))


async def _make_peer_calls(ids):
    combinations = list(itertools.combinations(ids, 2))
    logger.info('make-peer-call %s', combinations)

    tasks = []
    for id in ids:
        ids_to_call = [c[1] for c in combinations if c[0] == id]
        tasks.append(web_socket_client_manager.send_to(event_to_message('make-peer-call', ids_to_call), id))
    await asyncio.gather(*tasks)


async def handle_close_all_peer_connections(from_id):
    logger.info('close-all-room-peer-connections %s (%s)', from_id, await room_manager.get_client_name(from_id))
    room = await room_manager.get_room(from_id)

    if room is not None:
        await room_manager.send_to_all(event_to_message('close-all-peer-connections-request-received'), room=room)


async def get_ice_servers(request: web.Request):
    if 'Authorization' in request.headers and _isAuthorized(request.headers['Authorization']):
        return web.json_response(ice_servers)
    else:
        return web.json_response([])


async def on_shutdown(app):
    await web_socket_client_manager.close_all()
    await room_manager.clear_all()


def _isAuthorized(user_password):
    return password is None or user_password == password


class ExpandUserPath:
    def __new__(cls, path: Union[str, Path]) -> Path:
        return Path(path).expanduser().resolve()


class Args:
    port: int
    password: str
    ice_servers: Path
    static_folder: Path
    follow_symlinks: bool
    certificate: Path
    key: Path
    log_level: int


def main(other_routes=None):
    if other_routes is None:
        other_routes = []

    parser = argparse.ArgumentParser(description='OpenTera WebRTC Signaling Server')
    parser.add_argument('--port', type=int, help='Choose the port', default=8080)
    parser.add_argument('--password', type=str, help='Choose the password', default=None)
    parser.add_argument('--ice_servers', type=ExpandUserPath, help='Choose the ice servers json file', default=None)
    parser.add_argument('--static_folder', type=ExpandUserPath, help='Choose the static folder', default=None)
    parser.add_argument('--follow_symlinks', action="store_true", help='Follow symlinks for static folder, SECURITY RISK')
    parser.add_argument('--certificate', type=ExpandUserPath, help='TLS certificate path', default=None)
    parser.add_argument('--key', type=ExpandUserPath, help='TLS private key path', default=None)
    parser.add_argument('--log_level', type=int, choices=[logging.CRITICAL, logging.ERROR,
        logging.WARNING, logging.INFO, logging.DEBUG], help='Log level value', default=logging.DEBUG)

    # Parse arguments
    args = parser.parse_args(namespace=Args())

    # Set logging level
    logger.setLevel(args.log_level)

    # Default = not using TLS
    using_tls = False

    # Test for certificates / key pair
    if args.certificate and args.key:
        using_tls = True
        logger.info('Using certificate: %s', args.certificate)
        logger.info('Using private key: %s', args.key)
    elif args.certificate or args.key:
        sys.exit('You must specify both certificate and key.')
    else:
        using_tls = False

    # Update global password
    global password
    password = args.password

    # Look for ice servers file
    if args.ice_servers is not None:
        with args.ice_servers.open() as file:
            global ice_servers
            ice_servers = json.load(file)

    # Create route to get iceservers
    app.add_routes([web.get('/iceservers', get_ice_servers),
                    web.get('/signaling', web_socket_handler)] + other_routes)

    # Create static route if required
    if args.static_folder is not None:
        app.add_routes([web.static('/', args.static_folder, follow_symlinks=args.follow_symlinks)])

    # Shutdown callback
    app.on_shutdown.append(on_shutdown)

    # Run app
    if using_tls:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(args.certificate, args.key)
        web.run_app(app, port=args.port, ssl_context=ssl_context, shutdown_timeout=2)
    else:
        web.run_app(app, port=args.port, shutdown_timeout=2)


if __name__ == '__main__':
    main()
