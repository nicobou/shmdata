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
import functools
import json
import os
import pyquid
import socketio
from typing import Any, Dict, List, Optional, Tuple, Union
import logging

# Server creation
# ===============

try:
    port = os.environ['WS_PORT']
except KeyError:
    port = 8000

app = web.Application()
sio = socketio.AsyncServer(logger=True,
                           async_mode='aiohttp',
                           cors_allowed_origins='*')
sio.attach(app)

sw: Optional[pyquid.Switcher] = None
loop: asyncio.BaseEventLoop = None

formatter = logging.Formatter('%(name)s: %(message)s')
sio.logger.handlers[0].setFormatter(formatter)


# Switcher Callbacks
# ==================

def on_quiddity_created(quid_id: int) -> None:
    """Switcher callback to notify clients of successful quiddity creation.

    This is called by Switcher whenever a quiddity is created and emits
    a `quiddity.created` event to all clients in the room.

    Arguments:
        quid_id {int} -- The identifier of the newly created `Quiddity`.
    """
    try:
        quid = sw.get_quid(quid_id)
        set_signal_subscriptions(quid)
        set_property_subscriptions(quid)

        asyncio.create_task(sio.emit('quiddity.created', get_quiddity_model(quid)))
    except Exception as e:
        sio.logger.exception(e)


def on_quiddity_deleted(quid_id: int) -> None:
    """Switcher callback to notify clients of successful quiddity removal.

    This is called by Switcher whenever a quiddity is removed and emits
    a `quiddity.deleted` event to all clients in the room.

    Arguments:
        quid_id {int} -- The identifier of the removed `Quiddity`.
    """
    asyncio.create_task(sio.emit('quiddity.deleted', quid_id))


def on_quiddity_updated(quid_id: int, updated: Optional[dict]) -> None:
    """Switcher callback to notify clients about how went the quiddity update

    This is called by Switcher whenever a quiddity is updated and emits
    a `quiddity.updated` event to all clients in the room.

    Arguments:
        quid_id {int} -- The identifier of the removed `Quiddity`.
        updated {Optional[dict]} -- The dictionnary of updated properties
    """
    asyncio.create_task(sio.emit('quiddity.updated', quid_id, updated))


def on_user_tree_grafted(tree_path: pyquid.InfoTree, quid_id: int) -> None:
    """Switcher callback to notify clients that the user_tree is grafted

    Arguments:
        tree_path {InfoTree} -- An `InfoTree` that contains the grafted path.
        quid_id {int} -- The identifier of the updated `Quiddity`.
    """
    quid = sw.get_quid(quid_id)
    path = tree_path.get()
    value = quid.get_user_tree().get(path)
    asyncio.run_coroutine_threadsafe(
        sio.emit('user_tree.grafted', (quid_id, path, value)), loop)


def on_user_tree_pruned(tree_path: pyquid.InfoTree, quid_id: int) -> None:
    """Switcher callback to notify clients that the user_tree is pruned

    Arguments:
        tree_path {InfoTree} -- An `InfoTree` that contains the pruned path.
        quid_id {int} -- The identifier of the updated `Quiddity`.
    """
    path = tree_path.get()
    asyncio.run_coroutine_threadsafe(
        sio.emit('user_tree.pruned', (quid_id, path)), loop)


def on_info_tree_grafted(tree_path: pyquid.InfoTree, quid_id: int) -> None:
    """Switcher callback to notify clients that the info_tree is updated

    Arguments:
        tree_path {InfoTree} -- An `InfoTree` that contains the updated path.
        quid_id {int} -- The identifier of the updated `Quiddity`.
    """
    if quid_id not in sw.list_ids():
        return
    quid = sw.get_quid(quid_id)
    path = tree_path.get()
    value = quid.get_info_tree_as_json(path)

    sio.logger.debug(f'`{quid_id}`\'s info tree is grafted from `{path}`')
    asyncio.run_coroutine_threadsafe(
        sio.emit('info_tree.grafted', (quid_id, path, value)), loop)


def on_info_tree_pruned(tree_path: pyquid.InfoTree, quid_id: int) -> None:
    """Switcher callback to notify clients that the info_tree is pruned

    Arguments:
        tree_path {InfoTree} -- An `InfoTree` that contains the updated path.
        quid_id {int} -- The identifier of the updated `Quiddity`.
    """
    if quid_id not in sw.list_ids():
        return
    path = tree_path.get()

    sio.logger.debug(f'`{quid_id}`\'s info tree is pruned from `{path}`')
    asyncio.run_coroutine_threadsafe(sio.emit('info_tree.pruned', (quid_id, path)), loop)


def on_property_updated(value: Any, prop_data: Dict[str, str]) -> None:
    """Switcher callback to notify clients of the successful property update.

    This is called by Switcher whenever a property is updated and emits
    a `property.updated` event to all clients in the room.

    Arguments:
        value {Any} -- The new value of the property
        prop_data {Dict[str, str]} -- A dictionnary that contains the name and
                                      identifier of the updated property along
                                      with the identifier of the quiddity it
                                      belongs to
    """
    try:
        quid_id, property_name, property_id = prop_data.values()

        asyncio.run_coroutine_threadsafe(
            sio.emit('property.updated', (
                quid_id, property_name, value, property_id)), loop)
    except KeyError as e:
        sio.logger.exception(e)


def on_nickname_updated(nickname: str, quid_id: int) -> None:
    """Switcher callback to notify clients of successful nickname update.

    This is called by Switcher whenever a nickname is updated and emits
    a `nickname.updated` event to all clients in the room.

    Arguments:
        nickname {str} -- The new nickname of the quiddity
        quid_id {int} -- The identifier of the updated quiddity
    """
    try:
        asyncio.create_task(
            sio.emit('nickname.updated', (
                quid_id, str(nickname).replace("\"", ""))))
    except KeyError as e:
        sio.logger.exception(e)


# Subscriptions
def set_signal_subscriptions(quid: pyquid.Quiddity) -> None:
    """
    Arguments:
        quid {pyquid.Quiddity} -- [description]
    """

    quiddity_signals = {
        'on-nicknamed': on_nickname_updated,
        'on-user-data-grafted': on_user_tree_grafted,
        'on-user-data-pruned': on_user_tree_pruned,
        'on-tree-grafted': on_info_tree_grafted,
        'on-tree-pruned': on_info_tree_pruned
    }

    for signal, method in quiddity_signals.items():
        if not quid.subscribe(signal, method, quid.id()):
            sio.logger.warn(f'Could not subscribe to "{signal}" '
                            f'signal of `{repr(quid)}`')
        else:
            sio.logger.info(f'Subscribed to "{signal}" '
                            f'signal of `{repr(quid)}`')


def set_property_subscriptions(quid: pyquid.Quiddity) -> None:
    for prop in json.loads(quid.get_info_tree_as_json(".property")):
        prop_data = {
            'quid_id': quid.id(),
            'property_name': prop['id'],
            'property_id': prop['prop_id']
        }

        if not quid.subscribe(prop_data['property_name'],
                              on_property_updated, prop_data):
            prop_name = prop_data['property_name']
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

    # Properties subscription for existing quiddities
    for quid in sw.quiddities:
        set_property_subscriptions(quid)

    # Switcher signals subscription
    sw.subscribe("on-quiddity-created", on_quiddity_created)
    sw.subscribe("on-quiddity-removed", on_quiddity_deleted)


def start_server() -> None:
    global loop
    if sw is None:
        set_switcher_instance(pyquid.Switcher('switcher-ws'))
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    web.run_app(app, port=port, loop=loop)


def get_quiddity_model(quid: pyquid.Quiddity) -> dict:
    """Get the full model of a quiddity

    Arguments:
        quid {pyquid.Quiddity} -- A quiddity object from pyquid

    Returns:
        dict -- The quiddity model as a dictionnary
    """
    model = json.loads(str(quid))

    model['nickname'] = quid.nickname()
    model['infoTree'] = json.loads(str(quid.get_info_tree_as_json()))
    model['connectionSpecs'] = json.loads(str(quid.get_connection_specs()))
    model['userTree'] = json.loads(str(quid.get_user_tree()))

    # the json model should contain empty trees instead of null values
    if model['userTree'] is None:
        model['userTree'] = {}

    return model

# Generic Event Handlers
# ======================


@sio.event
def connect(sid: str, environ: Dict[str, Any]) -> None:
    """Special event called automatically when a client connects to the server

    The `connect` event is an ideal place to perform user authentication,
    and any necessary mapping between user entities in the application and
    the `sid` that was assigned to the client.

    The `environ` argument is a dictionary in standard WSGI format containing
    the request information, including HTTP headers.

    The `auth` argument contains any authentication details passed
    by the client, or None if the client did not pass anything.

    After inspecting the request, the connect event handler can return False
    to reject the connection with the client.

    Decorators:
        sio.event

    Arguments:
        sid {str} -- The session identifier assigned to the client
        environ {Dict[str, Any]} -- A dictionary in standard WSGI format
                                    containing the request information,
                                    including HTTP headers
    """
    version = (environ['QUERY_STRING']
               .split('&')[0].split('=')[-1] or 'Undefined')
    sio.enter_room(sid, 'web-clients')
    sio.logger.info(f'--- New client connected // Version: {version}')


@sio.event
def disconnect(sid: str) -> None:
    """Special event called automatically when a client disconnects

    Decorators:
        sio.event

    Arguments:
        sid {str} -- The session identifier assigned to the client
    """
    sio.leave_room(sid, 'web-clients')
    sio.logger.info(f'Client disconnected (SID: {sid})')


# Switcher API
# ============

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
def get_switcher_quiddities(sid: str) -> Tuple[Optional[str], List[dict]]:
    """Retrieves a list of existing quiddities.

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quids_list = [json.loads(str(quid)) for quid in sw.quiddities]
        return None, quids_list
    except Exception as e:
        sio.logger.exception(e)
        return str(e), []


@sio.on('switcher.bundles')
async def set_switcher_bundles(sid: str, bundles: Union[str, dict]) -> Tuple[Optional[str], bool]:
    """Set Switcher bundles from a json encoded string description

    The description of bundles must be **valid** as specified
    in [Writing Bundles](https://gitlab.com/sat-metalab/switcher/-/blob/master/doc/writing-bundles.md).

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        bundles {str|dict} -- The json encoded description of the bundles

    Returns:
        tuple -- The error and response for this event
    """
    try:
        if sw.load_bundles(json.loads(bundles) if isinstance(bundles, str) else bundles):
            return None, True
        else:
            return "Could not load bundle configuration", False

    except Exception as e:
        sio.logger.exception(e)
        return "Failed to load bundle configuration", False


@sio.on('switcher.log')
def switcher_log(sid: str, log_level: str, msg: str) -> Tuple[Optional[str], bool]:
    try:
        logger_method = getattr(sw.logger, log_level)
        logger_method(msg)
        return None, True
    except Exception as e:
        sio.logger.exception(e)
        return str(e), False


@sio.on('switcher.extra_config.paths')
def get_extra_config_paths(sid: str) -> Tuple[Optional[str], List[str]]:
    """Get the paths of the extra configuration

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    try:
        paths = sw.list_extra_configs()
        return None, paths
    except Exception as e:
        sio.logger.exception(e)
        return str(e), False


@sio.on('switcher.extra_config.read')
def read_extra_config(sid: str, name: str) -> Tuple[Optional[str], str]:
    """Read the extra configuration

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        name {str} -- The name of the configuration file to read

    Returns:
        tuple -- The error and response for this event
    """
    try:
        config = sw.read_extra_config(name)
        return None, config
    except Exception as e:
        sio.logger.exception(e)
        return str(e), False


# Quiddity API
# ============

@sio.on('quiddity.create')
def create_quiddity(
    sid: str,
    kind: str,
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
        nickname {Optional[str]} -- [description] (default: {None})
        properties {Optional[Dict[str, Any]]} -- [description] (default: {None})
        user_data {Optional[Dict[str, Any]]} -- [description] (default: {None})

    Returns:
        tuple -- The error and response for this event
    """
    try:
        quid = sw.create(kind, nickname or '')

        # props
        if properties is not None:
            for prop, value in properties.items():
                quid.set(prop, value)

        # user_data
        if user_data is not None:
            for key, sub_tree in user_data.items():
                tree = pyquid.InfoTree(json.dumps(sub_tree))
                quid.get_user_tree().graft(key, tree)

            sw.get_quid(quid.id()).notify_user_tree_grafted('.')

        return None, get_quiddity_model(quid)
    except Exception as e:
        sio.logger.exception(e)
        return f'{e} for `{kind}` kind', None


@sio.on('quiddity.delete')
def remove_quiddity(
    sid: str,
    quid_id: int
) -> Tuple[Optional[str], Optional[bool]]:
    """Remove a quiddity instance

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier

    Returns:
        tuple -- The error and response for this event
    """
    if sw.remove(quid_id):
        return None, True
    return f'Could not remove quiddity identified by `{quid_id}`', False


@sio.on('quiddity.connect')
def connect_quiddity(
    sid: str,
    src_id: int,
    dst_id: int
) -> Tuple[Optional[str], bool]:
    """Connect source and destination quiddities together

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
        sio.logger.exception(e)
        return f"Could not connect quiddities `{src.nickname()}` and `{dst.nickname()}`", False


@sio.on('quiddity.invoke')
async def invoke_quiddity_method(
    sid: str,
    quid_id: int,
    method_name: str,
    method_args: List[str] = []
) -> Tuple[Optional[str], Optional[Any]]:
    """Invoke a method of a quiddity

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


# Tree API
# ========

@sio.on('info_tree.get')
def get_quiddity_info_tree(sid: str, quid_id: int) -> Tuple[Optional[str], Optional[str]]:
    """Retrieves the info tree (read only) of a quiddity.

   Decorators:
        sio.on

   Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier

   Returns:
        tuple -- The error and response for this event
    """
    try:
        info_tree = sw.get_quid(quid_id).get_info_tree_as_json()
        return None, json.loads(str(info_tree))
    except Exception as e:
        sio.logger.exception(e)
        return str(e), None


@sio.on('user_tree.get')
def get_quiddity_user_tree(sid: str, quid_id: int) -> Tuple[Optional[str], Optional[str]]:
    """Retrieves the user tree of a quiddity.

   Decorators:
        sio.on

   Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier

   Returns:
        tuple -- The error and response for this event
    """
    try:
        user_tree = sw.get_quid(quid_id).get_user_tree()

        if user_tree == 'null':
            user_tree = {}

        return None, json.loads(str(user_tree))
    except Exception as e:
        sio.logger.exception(e)
        return str(e), None


@sio.on('user_tree.graft')
def graft_quiddity_user_tree(sid: str, quid_id: int, path: str, value: str) -> Tuple[Optional[str], Optional[str]]:
    """Graft the user tree of a quiddity.

   Decorators:
        sio.on

   Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        path {str} -- The path to graft
        value {str} -- The new value of the grafted path

   Returns:
        tuple -- The error and response for this event
    """
    try:
        sw.get_quid(quid_id).get_user_tree().graft(path, value)
        sw.get_quid(quid_id).notify_user_tree_grafted(path)
        user_tree = sw.get_quid(quid_id).get_user_tree()
        return None, json.loads(str(user_tree))
    except Exception as e:
        sio.logger.exception(e)
        return str(e), None


@sio.on('user_tree.prune')
def prune_quiddity_user_tree(sid: str, quid_id: int, path: str) -> Tuple[Optional[str], Optional[str]]:
    """Prune the user tree of a quiddity.

   Decorators:
        sio.on

   Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
        path {str} -- The path to graft

   Returns:
        tuple -- The error and response for this event
    """
    try:
        sw.get_quid(quid_id).get_user_tree().prune(path)
        sw.get_quid(quid_id).notify_user_tree_pruned(path)
        user_tree = sw.get_quid(quid_id).get_user_tree()
        return None, json.loads(str(user_tree))
    except Exception as e:
        sio.logger.exception(e)
        return str(e), None


@sio.on('connection_specs.get')
def get_quiddity_connection_specs(sid: str, quid_id: int) -> Tuple[Optional[str], Optional[str]]:
    """Retrieves the connection specs (read only) of a quiddity.

   Decorators:
        sio.on

   Arguments:
        sid {str} -- The session identifier assigned to the client
        quid_id {int} -- The quiddity identifier
   Returns:
        tuple -- The error and response for this event
    """
    try:
        connection_specs = sw.get_quid(quid_id).get_connection_specs()
        return None, json.loads(str(connection_specs))
    except Exception as e:
        sio.logger.exception(e)
        return str(e), None


# Property API
# ============

@sio.on('property.get')
async def get_property(
    sid: str,
    quid_id: int,
    property_name: str
) -> Tuple[Optional[str], Optional[Any]]:
    """Get the value of a property for a given quiddity

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


@sio.on('property.set')
async def set_property(
    sid: str,
    quid_id: int,
    property_name: str,
    value: str
) -> Tuple[Optional[str], Optional[bool]]:
    """Set the value of a property for a given quiddity

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
            return None, quid.get(property_name)
        return (f'Could not set value `{value}`'
                f'for property `{property_name}`', None)
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', None


# Nickname API
# ============

@sio.on('nickname.get')
async def get_nickname(
    sid: str,
    quid_id: int
) -> Tuple[Optional[str], Optional[str]]:
    """Get a given quiddity nickname

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
        return (f'Could not set nickname `{nickname}` '
                f'for quiddity `{quid_id}`'), False
    except KeyError:
        return f'Quiddity `{quid_id}` does not exist', False


# Session API
# ===========

@sio.on('session.reset')
def reset_session(sid: str):
    """Reset initial state for saving. When loading a new state,
    quiddities created after a call to reset_state will be cleared.

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    try:
        sw.reset_state()
        return None, True
    except Exception as e:
        sio.logger.exception(e)
        return str(e), False


@sio.on('session.clear')
def clear_session(sid: str):
    """Remove all Quiddities of the current session.

    Decorators:
        sio.on

    Arguments:
        sid {str} -- The session identifier assigned to the client

    Returns:
        tuple -- The error and response for this event
    """
    try:
        for qid in sw.list_ids():
            sw.remove(qid)
        return None, True
    except Exception as e:
        sio.logger.exception(e)
        return str(e), False


if __name__ == '__main__':
    start_server()
