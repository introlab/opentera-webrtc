import argparse
import asyncio
import itertools
import json
import sys
import logging

from pathlib import Path
from typing import Union

from aiohttp import web
from aiohttp_index import IndexMiddleware

import socketio
import ssl

from room_manager import RoomManager

PROTOCOL_VERSION = 1
DISCONNECT_DELAY_S = 1
INACTIVE_DELAY_S = 5

logging.basicConfig(format='[%(asctime)s] -- %(message)s')
logger = logging.getLogger('signaling_server')

sio = socketio.AsyncServer(async_mode='aiohttp', logger=False, engineio_logger=False, cors_allowed_origins='*')
app = web.Application(middlewares=[IndexMiddleware()])

room_manager = RoomManager(sio)

password = None
ice_servers = []


async def disconnect_delayed(id):
    await asyncio.sleep(DISCONNECT_DELAY_S)
    await sio.disconnect(id)


async def disconnect_inactive_user(id):
    await asyncio.sleep(INACTIVE_DELAY_S)
    # Verify if id joined a room
    room = await room_manager.get_room(id)
    if room is None:
        logger.info('inactive: %s', id)
        await sio.disconnect(id)


@sio.on('connect')
async def connect(id, env):
    logger.info('connect %s', id)
    # Launch task to verify if client joins a room (active), otherwise disconnect client.
    asyncio.create_task(disconnect_inactive_user(id))


@sio.on('disconnect')
async def disconnect(id):
    logger.info('disconnect %s (%s)', id, await room_manager.get_client_name(id))
    room = await room_manager.get_room(id)
    await room_manager.remove_client(id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        await room_manager.send_to_all('room-clients', data=clients, room=room)


@sio.on('join-room')
async def join_room(id, data):
    logger.info('join_room %s (%s)', id, data)

    if not _isAuthorized(data['password'] if 'password' in data else ''):
        asyncio.create_task(disconnect_delayed(id))
        return False
    if (data['protocolVersion'] if 'protocolVersion' in data else 0) != PROTOCOL_VERSION:
        asyncio.create_task(disconnect_delayed(id))
        return False

    if 'data' not in data:
        data['data'] = {}

    await room_manager.add_client(id, data['name'], data['data'], data['room'])

    clients = await room_manager.list_clients(data['room'])
    await room_manager.send_to_all('room-clients', data=clients, room=data['room'])

    return True


@sio.on('send-ice-candidate')
async def ice_candidate(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('send-ice-candidate %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('ice-candidate-received', data, to=data['toId'])


@sio.on('call-peer')
async def call_peer(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('call %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('peer-call-received', data, to=data['toId'])


@sio.on('make-peer-call-answer')
async def make_call_answer(from_id, data):
    from_name = await room_manager.get_client_name(from_id)
    to_name = await room_manager.get_client_name(data['toId'])
    logger.info('make-peer-call-answer %s (%s) to %s (%s)', from_id, from_name, data['toId'], to_name)
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('peer-call-answer-received', data, to=data['toId'])


@sio.on('call-all')
async def call_all(from_id):
    logger.info('call-all %s (%s)', from_id, await room_manager.get_client_name(from_id))
    room = await room_manager.get_room(from_id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        ids = [client['id'] for client in clients]
        await _make_peer_calls(ids)


@sio.on('call-ids')
async def call_ids(from_id, ids):
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
        tasks.append(sio.emit('make-peer-call', ids_to_call, to=id))
    await asyncio.wait(tasks)


@sio.on('close-all-room-peer-connections')
async def close_all_peer_connections(from_id):
    logger.info('close-all-room-peer-connections %s (%s)', from_id, await room_manager.get_client_name(from_id))
    room = await room_manager.get_room(from_id)

    if room is not None:
        await room_manager.send_to_all('close-all-peer-connections-request-received', room=room)


async def get_ice_servers(request: web.Request):
    if 'Authorization' in request.headers and _isAuthorized(request.headers['Authorization']):
        return web.json_response(ice_servers)
    else:
        return web.json_response([])


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
    socketio_path: str
    certificate: Path
    key: Path
    log_level: int
    robot_type: str


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OpenTera WebRTC Signaling Server')
    parser.add_argument('--port', type=int, help='Choose the port', default=8080)
    parser.add_argument('--password', type=str, help='Choose the password', default=None)
    parser.add_argument('--ice_servers', type=ExpandUserPath, help='Choose the ice servers json file', default=None)
    parser.add_argument('--static_folder', type=ExpandUserPath, help='Choose the static folder', default=None)
    parser.add_argument('--socketio_path', type=str, help='Choose the socketio path', default='socket.io')
    parser.add_argument('--certificate', type=ExpandUserPath, help='TLS certificate path', default=None)
    parser.add_argument('--key', type=ExpandUserPath, help='TLS private key path', default=None)
    parser.add_argument('--log_level', type=int, choices=[logging.CRITICAL, logging.ERROR,
        logging.WARNING, logging.INFO, logging.DEBUG], help='Log level value', default=logging.DEBUG)
    parser.add_argument('--robot_type', type=str, choices=['demo', 'ttop', 'beam'], help='Choose the robot type', default='demo')

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
    password = args.password

    # Look for ice servers file
    if args.ice_servers is not None:
        with args.ice_servers.open() as file:
            ice_servers = json.load(file)

    # Make sure websocket path is defined
    sio.attach(app, socketio_path=args.socketio_path)

    # Create route to get iceservers
    app.add_routes([web.get('/iceservers', get_ice_servers)])

    # Create static route if required
    if args.static_folder is not None:
        app.add_routes([web.static('/', args.static_folder)])

    # Run app
    if using_tls:
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(args.certificate, args.key)
        web.run_app(app, port=args.port, ssl_context=ssl_context)
    else:
        web.run_app(app, port=args.port)
