import argparse
import asyncio
import itertools
import json

from aiohttp import web

import socketio

from room_manager import RoomManager


sio = socketio.AsyncServer(async_mode='aiohttp')
app = web.Application()
sio.attach(app)

room_manager = RoomManager(sio)

password = None
ice_servers = []


def isAuthorized(user_password):
    return password is None or user_password == password


@sio.on('connect')
async def connect(id, env):
    print('connect ', id)


@sio.on('disconnect')
async def disconnect(id):
    print('disconnect ', id)
    room = await room_manager.get_room(id)
    await room_manager.remove_client(id)
    
    if room != None:
        clients = await room_manager.list_clients(room)
        await room_manager.send_to_all('room-clients', clients, room=room)


@sio.on('join-room')
async def join_room(id, data):
    print('join_room ', id, data)

    if not isAuthorized(data['password'] if 'password' in data else ''):
        return False

    await room_manager.add_client(id, data['name'], data['room'])

    clients = await room_manager.list_clients(data['room'])
    await room_manager.send_to_all('room-clients', clients, room=data['room'])

    return True


@sio.on('send-ice-candidate')
async def ice_candidate(from_id, data):
    #print('send-ice-candidate ', from_id, 'to', data['toId'])
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('ice-candidate-received', data, to=data['toId'])


@sio.on('call')
async def call(from_id, data):
    print('call ', from_id, 'to', data['toId'])
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('call-received', data, to=data['toId'])


@sio.on('call-all')
async def call(from_id):
    print('call-all', from_id)
    room = await room_manager.get_room(from_id)

    if room is not None:
        clients = await room_manager.list_clients(room)
        ids = [client['id'] for client in clients]
        combinations = list(itertools.combinations(ids, 2))
        
        tasks = []
        for id in ids:
            ids_to_call = [c[1] for c in combinations if c[0] == id]
            tasks.append(sio.emit('make-calls', ids_to_call, to=id))
        await asyncio.wait(tasks)


@sio.on('make-call-answer')
async def make_call_answer(from_id, data):
    print('make-call-answer ', from_id, 'to', data['toId'])
    room1 = await room_manager.get_room(from_id)
    room2 = await room_manager.get_room(data['toId'])

    if room1 is not None and room1 == room2:
        data['fromId'] = from_id
        await sio.emit('call-answer-received', data, to=data['toId'])


async def get_ice_servers(request):
    if 'Authorization' in request.headers and isAuthorized(request.headers['Authorization']):
        return web.json_response(ice_servers)
    else:
        return web.json_response([])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OpenTera WebRTC Signalling Server')
    parser.add_argument('--port', type=int, help='Choose the port', default=8080)
    parser.add_argument('--password', type=str, help='Choose the password', default=None)
    parser.add_argument('--ice_servers', type=str, help='Choose the ice servers json file', default=None)
    parser.add_argument('--static_folder', type=str, help='Choose the static folder', default=None)
    args = parser.parse_args()


    password = args.password

    if args.ice_servers is not None:
        with open(args.ice_servers) as file:
            ice_servers = json.load(file)


    app.add_routes([web.get('/iceservers', get_ice_servers)])
    if args.static_folder is not None:
        app.add_routes([web.static('/', args.static_folder)])
    web.run_app(app, port=args.port)
