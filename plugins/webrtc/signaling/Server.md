## Terminology

### Peer
 
Any webrtc application that can participate in a call

### Signaling server

Here it's a basic websockets server that manages the peers and rooms lists,
and shovels data between peers

## Overview

This is a basic protocol for doing 1-1 audio+video calls between webrtc peers.
The peers connect to the server for discovery and signaling.

### Steps in a typical scenario

To use this server, a webrtc application can follow these steps:

1. Connect to the websocket server
2. Register on the server
3. Join a room or contact a peer directly for a 1-1 session
4. Send signaling messages to the interesting peers through the server
5. Wait for new events in the room
6. The rest (transmission of a/v or media) is done p2p

## Connecting to the server

Connecting to the websocket server is just a matter of creating a websocket
connection object with your favourite language/framework.

**Javascript example**
```js
// connects to wss://localhost:8443
serverConnection = new WebSocket('wss://' + window.location.hostname + ':8443');
serverConnection.onmessage = wsMessageReceived;
serverConnection.onopen = wsConnected;
```

**Possible issues**
- Certificate: When the server is using a certificate generated locally,
  some web browsers might be unable to access the websocket server until the
  certificate has been reviewed and accepted. To do so open, using the above
  example, open `http://localhost:8443` in the browser and accept the certificate.

## Register on the server

Once the connection to the server has been established, the peer has to be
registered on server by sending it **`HELLO <username>`** through the websocket
connection. The server will answer **`HELLO`** if the registration succeeds.

**Message**
- `HELLO <username>`: Register me as `<username>`.

**Response**
- `HELLO `: Everything went well.

**Constraints**
- `<username>` can't be empty.
- `<username>` can't contain whitespaces.
- `<username>` must not be already used by another peer.

If any of the constraints are violated, the server will simply close the 
connection.

## Start/join a call

The server supports two types of call: 1-1 calls, and group calls.

### 1-1 call

* To connect to a single peer, send `SESSION <uid>` where `<uid>` identifies the peer to connect to, and receive `SESSION_OK`
* All further messages will be forwarded to the peer
* The call negotiation with the peer can be started by sending JSON encoded SDP and ICE

* Closure of the server connection means the call has ended; either because the other peer ended it or went away
* To end the call, disconnect from the server. You may reconnect again whenever you wish.

### Party call

To start or join a group call, the client has to join a room on the server and
then negotiate with the other peers in it.

#### Joining room

**Message**
- `ROOM <id>`: If the room `<id>` doesn't exist, create it. Add
  me to room.

**Response**
- `ROOM_OK [peer ...]`: The peer has been successfully added to the room.
  `[peer ...]` is an _optional_ list of peers already in the room.
- `ERROR invalid room id <id>`

**Constraints**
- `<id>` can't be empty.
- `<id>` can't contain whitespaces.
- `<id>` can't be `session`.

## Message a peer

Messages can be sent from one peer to another through the server, for example for
signaling.


**Message**
- `ROOM_PEER_MSG <peer> <data>`: Send `<data>` to `<peer>`.

**Response**
- `ERROR peer <peer> not found`:  Peer is not connected on the server.
- `ERROR peer <peer> is not in room`: Peer is not in the same room as the messenger.

**Constraints**
- `<peer>` must be registered on the server.
- `<peer>` must be in the same room as the `<messenger>`.

If all constraints are respected, at the other end of the connection, 
the `<peer>` will receive a message saying: `ROOM_PEER_MSG <messenger> <data>`, 
where `<messenger>` is the author of the message.

## Query the server

The server can be queried to retrieve some information.

### Get the list of peers in a room

Peers do not need to use this method since they are guaranteed to always have
an up to date list of the members of the room they are in. When a peer joins a
room, it receives the list of users in the room, and evey user in the room 
receives an event announcing the new member.

**Message**
- `ROOM_PEER_LIST`

**Response**
- `ROOM_PEER_LIST [peer ...]`: `[peer ...]` is a list of all the peers in the room. It can be empty.

**Constraints**
- You must be in a group call.

## Events

- `ROOM_PEER_JOINED <peer>`: `<peer>` joined the room. 


## Negotiation

Peers exchange SDP and ICE information by sending messages to each other through the
server. The messages are in the form `ROOM_PEER_MSG <peer> <data>`, where `<data>` is
a JSON object. See Signaling.md for more information about the content of that 
object.
