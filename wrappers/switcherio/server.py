#!/usr/bin/env python3

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

from aiohttp import web
import asyncio
import json
import os
import pyquid
import socketio
from typing import Any, Dict, List, Optional, Tuple
import logging

try:
    port = os.environ['WS_PORT']
except KeyError:
    port = 8000

# Server creation
app = web.Application()
sio = socketio.AsyncServer(logger=True, async_mode='aiohttp', cors_allowed_origins='*')
sio.attach(app)

sw: Optional[pyquid.Switcher] = None
loop: asyncio.BaseEventLoop = None

formatter = logging.Formatter('%(name)s: %(message)s')
sio.logger.handlers[0].setFormatter(formatter)

# Switcher Callbacks


def on_quiddity_created(quid_id: int) -> None:
    """Switcher callback signal to notify all clients of the successful creation of the quiddity.

    This is called by Switcher whenever a quiddity is created and emits a `quiddity.created` event
    to all clients in the room.

    Arguments:
        quid_id {int} -- The identifier of the newly created `Quiddity`.
    """
    quid = sw.get_quid(quid_id)

    set_signal_subscriptions(quid)
    set_property_subscriptions(quid)

    asyncio.create_task(sio.emit('quiddity.created',
                                 quid.get_info_tree_as_json()))


def on_quiddity_removed(quid_id: int) -> None:
    """Switcher callback signal to notify all clients of the successful removal of the quiddity.

    This is called by Switcher whenever a quiddity is removed and emits a `quiddity.removed` event
    to all clients in the room.

    Arguments:
        quid_id {int} -- The identifier of the removed `Quiddity`.
    """
    asyncio.create_task(sio.emit('quiddity.removed', quid_id))


def on_property_updated(value: Any, user_data: Dict[str, str]) -> None:
    """Switcher callback signal to notify all clients of the successful update of a property.

    This is called by Switcher whenever a property is updated and emits a `property.updated` event
    to all clients in the room.

    Arguments:
        value {Any} -- The new value of the property
        user_data {Dict[str, str]} -- A dictionnary that contains the name and identifier of the updated property along with the identifier of the quiddity it belongs to
    """
    try:
        quid_id = user_data['quid_id']
        property_name = user_data['property_name']
        property_id = user_data['property_id']
        asyncio.run_coroutine_threadsafe(
            sio.emit('property.updated', (
                quid_id, property_name, value, property_id)), loop)
    except KeyError as e:
        sio.logger.warn(f'Error in "on_property_updated" callback: {e}')


def on_nickname_updated(nickname: str, quid_id: int) -> None:
    """Switcher callback signal to notify all clients of the successful update of a nickname.

    This is called by Switcher whenever a nickname is updated and emits a `nickname.updated` event
    to all clients in the room.

    Arguments:
        nickname {str} -- The new nickname of the quiddity
        quid_id {int} -- The identifier of the updated quiddity
    """
    try:
        asyncio.create_task(
            sio.emit('nickname.updated', (
                quid_id, str(nickname).replace("\"", ""))))
    except KeyError as e:
        sio.logger.warn(f'Error in "on_nickname_updated" callback: {e}')


# Subscriptions
def set_signal_subscriptions(quid: pyquid.Quiddity) -> None:
    """

    Arguments:
        quid {pyquid.Quiddity} -- [description]
    """
    if not quid.subscribe('on-nicknamed', on_nickname_updated, quid.id()):
        sio.logger.warn('Could not subscribe to "on-nicknamed" '
                        f'signal of `{repr(quid)}`')


def set_property_subscriptions(quid: pyquid.Quiddity) -> None:
    user_data = {}
    user_data['quid_id'] = quid.id()
    for prop in json.loads(quid.get_info_tree_as_json(".property")):
        user_data['property_name'] = prop['id']
        user_data['property_id'] = prop['prop_id']
        if not quid.subscribe(user_data['property_name'],
                              on_property_updated, user_data):
            prop_name = user_data['property_name']
            sio.logger.warn(f'Could not subscribe to property `{prop_name}` '
                            f'on `{repr(quid)}`')


def remove_property_subscriptions(quid: pyquid.Quiddity) -> None:
    props = json.loads(quid.get_info_tree_as_json(".property"))
    for prop in props:
        if not quid.unsubscribe(prop['id']):
            prop_id = prop['id']
            sio.logger.warn(f'Could not unsubscribe from property "{prop_id}" '
                            f'on `{repr(quid)}`')


# Utilities
def set_switcher_instance(instance: pyquid.Switcher) -> None:
    global sw
    if sw is not None:
        sw.unsubscribe("on-quiddity-created")
        sw.unsubscribe("on-quiddity-removed")

    sw = instance

    # Properties subscription for existng quiddities
    for quid in sw.quiddities:
        set_property_subscriptions(quid)

    # Switcher signals subscription
    sw.subscribe("on-quiddity-created", on_quiddity_created)
    sw.subscribe("on-quiddity-removed", on_quiddity_removed)


def start_server() -> None:
    global loop
    if sw is None:
        set_switcher_instance(pyquid.Switcher('switcher-ws'))
    loop = asyncio.get_event_loop()
    web.run_app(app, port=port)


# Generic Event Handlers
@sio.event
def connect(sid: str, environ: Dict[str, Any]) -> None:
    """Special event called automatically when a client connects to the server

    The `connect` event is an ideal place to perform user authentication, and any
    necessary mapping between user entities in the application and the `sid` that
    was assigned to the client.

    The `environ` argument is a dictionary in standard WSGI format containing
    the request information, including HTTP headers.

    The `auth` argument contains any authentication details passed by the client,
    or None if the client did not pass anything.

    After inspecting the request, the connect event handler can return False
    to reject the connection with the client.

    Decorators:
        sio.event

    Arguments:
        sid {str} -- The session identifier assigned to the client
        environ {Dict[str, Any]} -- A dictionary in standard WSGI format containing the request information, including HTTP headers
    """
    version = (environ['QUERY_STRING']
               .split('&')[0].split('=')[-1] or 'Undefined')
    sio.enter_room(sid, 'web-clients')
    sio.logger.info(f'--- New client connected // Version: {version}')


@sio.event
def disconnect(sid: str) -> None:
    """Special event called automatically when a client disconnects from the server

    Decorators:
        sio.event

    Arguments:
        sid {str} -- The session identifier assigned to the client
    """
    sio.leave_room(sid, 'web-clients')
    sio.logger.info(f'Client disconnected (SID: {sid})')


# Event Callbacks
@sio.on('switcher.name')
def get_switcher_name(sid: str) -> Tuple[None, str]:
    """Get the name of the Switcher instance

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    return None, sw.name()


@sio.on('switcher.version')
def get_switcher_version(sid: str) -> Tuple[None, str]:
    """Get the version of the Switcher instance

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    return None, sw.version()


@sio.on('switcher.kinds')
def get_switcher_kinds(sid: str) -> Tuple[None, str]:
    """Get documentation for all kinds of quiddities

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    return None, sw.kinds_doc()


@sio.on('switcher.quiddities')
def get_switcher_quiddities(sid: str) -> Tuple[None, str]:
    """Get documentation for all kinds of quiddities

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    return None, sw.quids_descr()


@sio.on('switcher.bundles')
async def set_switcher_bundles(sid: str, bundles: str) -> Tuple[Optional[str], Optional[bool]]:
    """Set Switcher bundles from a json encoded string description

    The description of the bundles must be **valid** as specified 
    in [Writing Bundles](https://gitlab.com/sat-metalab/switcher/-/blob/master/doc/writing-bundles.md).

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        bundles {str} -- The json encoded description of the bundles

    Returns:
        tuple -- The error and response for this event
    """
    if type(bundles) is str:
        if sw.load_bundles(bundles):
            return None, True
        return 'Could not load bundle configuration', None
    return '"bundles" argument is mandatory and must be a string', None


# Quiddity API
@sio.on('quiddity.create')
def create_quiddity(
    sid: str,
    kind: str,
    name: Optional[str] = None,
    nickname: Optional[str] = None,
    properties: Optional[Dict[str, Any]] = None,
    user_data: Optional[Dict[str, Any]] = None
) -> Tuple[Optional[str], Optional[str]]:
    """Create a quiddity instance

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        kind {str} -- The kind of quiddity to create

    Keyword Arguments:
        name {Optional[str]} -- [description] (default: {None})
        nickname {Optional[str]} -- [description] (default: {None})
        properties {Optional[Dict[str, Any]]} -- [description] (default: {None})
        user_data {Optional[Dict[str, Any]]} -- [description] (default: {None})

    Returns:
        tuple -- The error and response for this event
    """
    name = name or ''
    try:
        quid = sw.create(kind, name)
    except Exception as e:
        return f'{e} for `{kind}` kind', None

    if quid is not None:

        if properties is not None:
            for prop, value in properties:
                quid.set(prop, value)

        if user_data is not None:
            tree = pyquid.InfoTree(json.dumps(user_data))
            quid.get_user_tree().graft('.', tree)

        if nickname is not None:
            quid.set_nickname(nickname)

        return None, quid.get_info_tree_as_json()


@sio.on('quiddity.connect')
def connect_quiddity(
    sid: str,
    src_id: int,
    dst_id: int
) -> Tuple[Optional[str], Optional[str]]:
    """Connect source and destination quiddities together

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        src_id {int} -- The source quiddity identifier to connect to the destination
        dst_id {int} -- The destination quiddity identifier to connect the source to

    Returns:
        tuple -- The error and response for this event
    """
    src = sw.get_quid(src_id)
    dst = sw.get_quid(dst_id)
    try:
        src.try_connect(dst)
        return None, True
    except Exception as e:
        sio.logger.warn(e)
        return f"Could not connect quiddities `{src.nickname()}` and `{dst.nickname()}`", False


@sio.on('quiddity.remove')
def remove_quiddity(
    sid: str,
    nickname: str
) -> Tuple[Optional[str], Optional[bool]]:
    """Remove a quiddity instance

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        nickname {str} -- The nickname of the quiddity to remove

    Returns:
        tuple -- The error and response for this event
    """
    quid_id = sw.get_quid_id(nickname)
    if sw.remove(quid_id):
        return None, True
    return f'Could not remove `{nickname}` quiddity', None


# Property API
@sio.on('property.get')
async def get_property(
    sid: str,
    quid_id: int,
    property_name: str
) -> Tuple[Optional[str], Optional[Any]]:
    """Get the value of a property for a given quiddity

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        property_name {str} -- The name of the property to get

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.get_quid(quid_id)
        return None, quid.get(property_name)
    except ValueError as e:
        return f'Could not get property value: `{e}`', None
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None

# @NOTE: this function should be rewritten
@sio.on('property.set')
async def set_property(
    sid: str,
    quid_id: int,
    property_name: str,
    value: str
) -> Tuple[Optional[str], Optional[bool]]:
    """Set the value of a property for a given quiddity

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        property_name {str} -- The name of the property to get
        value {str} -- The value to assign to the property

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.get_quid(quid_id)
        if quid.set(property_name, value):
            return None, True
        return (f'Could not set value `{value}`'
                f'for property `{property_name}`', None)
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None


# Method API
@sio.on('method.invoke')
async def invoke_method(
    sid: str,
    quid_id: int,
    method_name: str,
    method_args: List[str] = []
) -> Tuple[Optional[str], Optional[Any]]:
    """Invoke a method of a quiddity

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        method_name {str} -- The name of the method to invoke
        method_args {List[str] = []} -- The list of arguments for the method

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.get_quid(quid_id)
        return None, quid.invoke(method_name, method_args)
    except TypeError as e:
        return f'Could not invoke method `{e}`', None
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None


# Nickname API
@sio.on('nickname.get')
async def get_nickname(
    sid: str,
    quid_id: int
) -> Tuple[Optional[str], Optional[str]]:
    """Get a given quiddity nickname

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.get_quid(quid_id)
        return None, quid.nickname()
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None


@sio.on('nickname.set')
async def set_nickname(
    sid: str,
    quid_id: int,
    nickname: str
) -> Tuple[Optional[str], Optional[bool]]:
    """Set a given quiddity nickname

    [description]

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        nickname {str} -- The new nickname for the quiddity

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.get_quid(quid_id)
        if quid.set_nickname(nickname):
            return None, True
        return (f'Could not set nickname "{nickname}" '
                f'for quiddity "{quid_id}"', None)
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None


if __name__ == '__main__':
    start_server()
