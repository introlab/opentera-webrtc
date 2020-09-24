import argparse
import asyncio

from aiohttp import web

import socketio


sio = socketio.AsyncServer(async_mode='aiohttp')
app = web.Application()
sio.attach(app)


@sio.on('connect')
async def connect(socket_id, env):
    print('connect ', socket_id)


@sio.on('disconnect')
async def disconnect(socket_id):
    print('disconnect ', socket_id)

    room = await get_room(socket_id)
    if room is not None:
        room_socket_ids = get_room_socket_ids(room)
        room_socket_ids.remove(socket_id)
        if len(room_socket_ids) < 2:
            print(room, 'room not ready')
            await sio.emit('room-not-ready', '', room=room)


@sio.on('join-room')
async def join_room(socket_id, data):
    print('join_room ', socket_id, data)
    room_socket_ids = get_room_socket_ids(data['room'])
    
    if len(room_socket_ids) > 1:
        print('join_room refused')
        return False
    if len(room_socket_ids) == 1 and await get_client_type(room_socket_ids[0]) == data['client_type']:
        print('join_room refused')
        return False

    await sio.save_session(socket_id, {'client_type': data['client_type'], 'room': data['room']})
    sio.enter_room(socket_id, data['room'])

    room_socket_ids = get_room_socket_ids(data['room'])
    if len(room_socket_ids) == 2:
        print(data['room'], 'room ready')
        await sio.emit('room-ready', '', room=data['room'])
    else:
        print(data['room'], 'room not ready')
        await sio.emit('room-not-ready', '', room=data['room'])
    
    return True


@sio.on('ice-candidate')
async def ice_candidate(socket_id, candidate):
    print('ice_candidate ', socket_id, candidate)
    room = await get_room(socket_id)
    if room is not None:
        await sio.emit('ice-candidate', candidate, room=room, skip_sid=socket_id)


@sio.on('call')
async def call(socket_id, offer):
    print('call ', socket_id, offer)
    room = await get_room(socket_id)
    if room is not None:
        await sio.emit('call-received', offer, room=room, skip_sid=socket_id)


@sio.on('make-call-answer')
async def make_call_answer(socket_id, answer):
    print('answer ', socket_id, answer)
    room = await get_room(socket_id)
    if room is not None:
        await sio.emit('call-answer-received', answer, room=room, skip_sid=socket_id)


def get_room_socket_ids(room):
    if '/' in sio.manager.rooms and room in sio.manager.rooms['/']:
        return list(sio.manager.rooms['/'][room].keys())
    else:
        return []


async def get_client_type(socket_id):
    session = await sio.get_session(socket_id)
    return session['client_type']


async def get_room(socket_id):
    session = await sio.get_session(socket_id)
    if 'room' in session:
        return session['room']
    else:
        return None


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='OpenTera WebRTC Signalling Server')
    parser.add_argument('--port', type=int, help='Choose the port', default=8080)
    parser.add_argument('--static_folder', type=str, help='Choose the static folder', default=None)
    args = parser.parse_args()

    if args.static_folder is not None:
        app.add_routes([web.static('/', args.static_folder)])

    web.run_app(app, port=args.port)
