var localVideo;
var localStream;
var peers = {};
var ID;
var ROOM;
var serverConnection;

class Peer {
  constructor(id) {
    addLog(`${id}: Creating peer`);
    console.log(`${id}: Creating peer`);
    this.id = id;
    this.txStream = new MediaStream();
    this.rxStream = new MediaStream();
    this.rxIceQueue = [];
    this.txIceQueue = [];
    this.cacheTxIce = true;
    this.cacheRxIce = true;
    this.videoElement = null;
  }

  hangup() {
    addLog(`${this.id}: Hanging up`);
    console.log(`${this.id}: Hanging up`);
    if(this.connection) {
      this.connection.onicecandidate = null;
      this.connection.ontrack = null;
      this.connection.oniceconnectionstatechange = null;
      this.connection.onicesignalingstatechanged = null;
      this.connection.onicegatheringstatechanged = null; 
      this.connection.close()
      this.connection = null;
    }

    if(this.videoElement) {
      if (this.videoElement.srcObject) {
        this.videoElement.srcObject.getTracks().forEach(track => track.stop());
      }
      removeVideo(this.id);
    }

    addLog(`${this.id}: Call stopped`);
    console.log(`${this.id}: Call stopped`);
  }

  initPeerConnection() {
    addLog(`Setting up connection with ${this.id}`);
    console.log(`Setting up connection with ${this.id}`);
    this.connection = new RTCPeerConnection(Peer.connectionConfig);
    this.addStream(localStream);
    this.connection.onicecandidate = this.iceCandidateCollected.bind(this);
    this.connection.ontrack = this.newTrack.bind(this);
    this.connection.oniceconnectionstatechange = this.iceConnectionStateChanged.bind(this);
    this.connection.onicesignalingstatechanged = this.iceSignalingStateChanged.bind(this);
    this.connection.onicegatheringstatechanged = this.iceGatheringStateChanged.bind(this);
  }

  iceConnectionStateChanged(event) {
    const state = this.connection.iceConnectionState
    let message = `${this.id}: iceConnectionState changed to ${state}`;

    let log = null;
    switch(this.connection.iceConnectionState) {
      case "closed":
      case "failed":
      case "disconnected":
        log = console.warn;
        break;
      default:
        log = console.log;
        break;
    }  

    log(message);
  }

  iceSignalingStateChanged(event) {
    const state = this.connection.signalingState;
    let message = `${this.id}: ICE signalingState changed to ${state}`;

    let log = null;
    switch(state) {
      case "closed":
        log = console.warn;
        break;
      default:
        log = console.log;
        break;
    }

    log(message);
  }

  iceGatheringStateChanged(event) {
    console.log(`${this.id}: iceGatheringState changed to: ${this.connection.iceGatheringState}`);
  }

  addStream(stream) {
    addLog(`${this.id}: Adding streams`);
    console.log(`${this.id}: Adding streams`);
    for (const track of stream.getTracks()) {
      this.connection.addTrack(track, this.txStream);
    }
  }

  trackMuted(event) {
    console.log(`${this.id}: Track muted`);
  }

  trackEnded(event) {
    console.log(`${this.id}: Track ended`);
  }

  newTrack(event) {
    event.track.onmuted = this.trackMuted.bind(this);
    event.track.onended = this.trackEnded.bind(this);

    addLog(`${this.id}: New track`);
    console.log(`${this.id}: New track`);
    console.info("Track: ", this.id, event);

    if(!this.videoElement) {
      addLog(`${this.id}: Retrieving video element`);
      console.log(`${this.id}: Retrieving video element`);
      this.videoElement = addVideo(this.id, "controls");
      this.videoElement.srcObject = this.rxStream;
    }

    this.rxStream.addTrack(event.track);
  }

  remoteDescriptionSet(type) {
    console.log(`${this.id}: Remote description set`);
    addLog(`${this.id}: Remote description set`);
    
    this.cacheRxIce = false;
    while(this.rxIceQueue.length > 0) {
      //this.connection.addIceCandidate(this.rxIceQueue.shift());
      this.addCandidate(this.rxIceQueue.shift());
    }

    if (type === "offer") {
      this.incomingCall();
    } 
    //} else if (type === "answer") {
      this.cacheTxIce = false;
      while(this.txIceQueue.length > 0) {
        this.iceCandidateCollected({"candidate": this.txIceQueue.shift()}); 
      }
    //}
  }

  addCandidate(candidate) {
    if (this.cacheRxIce) {
      addLog(`${this.id}: (rx) Caching ICE candidate`);
      console.log(`${this.id}: (rx) Caching ICE candidate`);
      this.rxIceQueue.push(candidate); 
    } else {
      addLog(`${this.id}: Adding ICE candidate`);
      console.log(`${this.id}: Adding ICE candidate`);
      this.connection.addIceCandidate(candidate)
        .catch(function(err) {
          console.error(err);
        });
    }
  }

  iceCandidateCollected(event) {
    if (this.cacheTxIce) {
      console.log(`${this.id}: (tx) Caching ICE candidate`);
      addLog(`${this.id}: (tx) Caching ICE candidate`);
      this.txIceQueue.push(event.candidate);
    } else if(event.candidate == null || event.candidate.candidate === '') {
      console.log(`${this.id}: (tx) Ignoring nil ICE candidate`);
      addLog(`${this.id}: (tx) Ignoring nil ICE candidate`);
      return;
    } else {
      sendCandidate(this.id, event.candidate);
    }
  }

  localDescriptionSet() {
    addLog(`${this.id}: Local description set`);
    console.log(`${this.id}: Local description set`);

    sendDescription(this.id, this.connection.localDescription);
  }

  descriptionCreated(description) {
    addLog(`${this.id}: Local description created (${description.type})`);
    console.log(`${this.id}: Local description created (${description.type})`);

    const resolve = this.localDescriptionSet.bind(this);
    this.connection.setLocalDescription(description)
      .then(resolve)
      .catch(function(err) {
        console.error(err);
      });
  }

  incomingCall() {
    addLog(`${this.id} Incoming call`);
    console.log(`${this.id} Incoming call`);

    const resolve = this.descriptionCreated.bind(this);
    this.connection.createAnswer()
      .then(resolve)
      .catch(function(err) {
        console.error(err);
      });
  }

  startCall() {
    addLog(`${this.id}: Starting call`);
    console.log(`${this.id}: Starting call`);

    const resolve = this.descriptionCreated.bind(this);

    this.connection.createOffer({iceRestart: true})
      .then(resolve)
      .catch(function(err) {
        console.error(err);
      });
  }

  static checkSDP(message) {
    return message.hasOwnProperty("type") 
      && message.hasOwnProperty("sdp")
      && (message.type === "offer" || message.type === "answer");
  }

  static checkICE(message) {
    return message.hasOwnProperty("candidate")
      && message.hasOwnProperty("sdpMLineIndex");
  }

  static checkCommand(message) {
    return message.hasOwnProperty("type")
      && message.hasOwnProperty("command");
  }

  handleCustomCommand(command) {
    switch(command) {
      case "STOP CALL":
        addLog(`${this.id}: Asked to stop call`);
        console.log(`${this.id}: Asked to stop call`);
        this.hangup();
        this.txIceQueue = [];
        this.cacheTxIce = false;
        break;
      default:
        console.warn("Unknown command: [%s]", command);
        break;
    }
  }

  newMessage(message) {
    if (message.hasOwnProperty("command")) {
      const command = message.command;  
      if(!Peer.checkCommand(command)) {
        console.warn("Bad command message"); 
        return;
      }

      switch(command.type) {
        case "customCommand":
          this.handleCustomCommand(command.command);
          break;
        default:
          console.warn("Unknown command type: [%s]", command.type);
          break;
      }
    } else if (message.hasOwnProperty("sdp")) {
      const sdp = message.sdp;
      if (!Peer.checkSDP(sdp)) {
        console.warn("Bad SDP message");
        return;
      }

      addLog(`${this.id}: Received remote description (${sdp.type})`);
      console.log(`${this.id}: Received remote description (${sdp.type})`);

      if (!this.connection) {
        this.initPeerConnection(); 
      }

      const resolve = this.remoteDescriptionSet.bind(this, sdp.type);
      this.connection.setRemoteDescription(sdp)
        .then(resolve)
        .catch(function(err) {
          console.error(err); 
        })
    } else if (message.hasOwnProperty("ice")) {
      console.log(`${this.id}: (rx) New ICE candidate message`);
      addLog(`${this.id}: (rx) New ICE candidate message`);
      const ice = message.ice;
      if(!Peer.checkICE(ice)) {
        console.warn("Bad ICE message");
        return;
      }

      this.addCandidate(new RTCIceCandidate(ice));
    }
  }
}

Peer.connectionConfig = {
  'iceServers': [
    {'urls': 'stun:stun.stunprotocol.org:3478'},
    {'urls': 'stun:stun.l.google.com:19302'},
  ]
};

function sendMessage(message) {
  serverConnection.send(message);
}

function wsMessageReceived(message) {
  let content = message.data;

  if (content === "HELLO") {
    registeredWithServer();
    joinRoom(ROOM);
  } else if (content.startsWith("ROOM")) {
    if (content.startsWith("ROOM_OK")) {
      roomJoined();
      if(content === "ROOM_OK ") {
        return;
      }

      const users = content.split(" ").slice(1);
      for (const user of users) {
        addLog(`${user}: is in the room`);
        console.log(`${user}: is in the room`);

        addUser(user);
        peers[user] = new Peer(user);
        peers[user].initPeerConnection();
        peers[user].startCall();
      }
    } else if (content.startsWith("ROOM_PEER")) {
      const type = content.split(" ")[0];
      const sender = content.split(" ")[1];
      switch(type) {
        case "ROOM_PEER_MSG":
          if (!(sender in peers)) {
            peers[sender] = new Peer(sender);
            peers[sender].initPeerConnection();
          }
          const data = JSON.parse(content.slice(type.length + sender.length + 2));
          peers[sender].newMessage(data);
          break;
        case "ROOM_PEER_LEFT":
          if(sender in peers) {
            addLog(`${sender} left the room`);
            peers[sender].hangup();
            delete peers[sender];
          }
          break;
        case "ROOM_PEER_JOINED":
          addLog(`${sender} joined the room`);
          addUser(sender);
          break;
        default:
          console.warn("Unsupported message type: %s", type, message);
          break;
      }
    }
  } else if(content.startsWith("ERROR")) {
    addLog(content);
    console.warn(content);
  } else {
    console.warn("Unknown prefix. Can't parse message", message);
  }
}

function sendCandidate(target, candidate) {
  console.log(`${target}: Sending ICE candidate`);
  addLog(`${target}: Sending ICE candidate`);

  let cand = JSON.stringify({'ice': candidate});
  sendMessage(`ROOM_PEER_MSG ${target} ${cand}`);
}

function sendDescription(target, description) {
  console.log(`${target}: Sending SDP`);
  addLog(`${target}: Sending SDP`);

  let desc = JSON.stringify({'sdp': description, 'satid': "SAT_WebRTC_Client_Web" });
  sendMessage(`ROOM_PEER_MSG ${target} ${desc}`);
}

function roomJoined() {
  addLog(`Room joined`);
  addUser(ID);
}

function joinRoom(room) {
  addLog(`Joining room: ${room}`);
  sendMessage(`ROOM ${room}`);
}

function registeredWithServer() {
  addLog(`Registration completed`);
}

function registerWithServer() {
  addLog(`Registering as: ${ID}`);
  sendMessage(`HELLO ${ID}`);
}

function wsConnected(event) {
  addLog("Connection established");
  console.log("Connection established: ", event);
  registerWithServer();
}

function wsClosed(event) {
  addLog("WebSocket connection closed");
  console.log("WebSocket connection closed: ", event);
}

function initWSocket(server) {
  addLog(`Connecting to: ${server}`);
  serverConnection = new WebSocket(server);
  serverConnection.onmessage = wsMessageReceived;
  serverConnection.onopen = wsConnected;
  serverConnection.onclose = wsClosed;
}

function mdAcquirementError(error) {
  switch(error.name) {
    case "NotFoundError":
      alert("Unable to open your call because no camera and/or microphone" +
        "were found.");
      break;
    case "SecurityError":
    case "PermissionDeniedError":
      console.warn(error);
      break;
    default:
      alert("Error opening your camera and/or microphone: " + error.message);
      break;
  }

  // FIXME
  //closeVideoCall();
}

function connect() {
  if(serverConnection && serverConnection.readyState !== WebSocket.CLOSED) {
    addLog(`Disconnecting from server`);
  }

  const server = document.getElementById("server-input").value;
  const room = document.getElementById("room-input").value;
  const user = document.getElementById("user-input").value;

  ROOM = room;
  ID = user;
  initWSocket(server);
}

function mediaDevicesAcquired(stream) {
  addLog("Media devices aquired");
  localStream = stream;
  localVideo.srcObject = stream;
}

function acquireUserMedia(constraints) {
  if(navigator.mediaDevices.getUserMedia) {
    navigator.mediaDevices.getUserMedia(constraints)
      .then(mediaDevicesAcquired)
      .catch(function(err) {
        console.error(err);
      });
  } else {
    alert('Your browser does not support getUserMedia API');
  }
}

function addVideo(id, attributes) {
  var check = document.getElementById(`video-${id}`);
  if (check) {
    check.srcObject = null;
    return check;
  }

  const element = `<video id="video-${id}" autoplay style="width:40%;" ${attributes}></video>`;
  var cont = document.getElementById("videoContainer");
  cont.insertAdjacentHTML('afterbegin', element);
  return document.getElementById(`video-${id}`);
}

function removeVideo(id) {
  removeElement(`video-${id}`);
}

function removeElement(id) {
  document.getElementById(id).remove();
}

function initHTML() {
  localVideo = addVideo("localVideo", "controls muted");
  document.getElementById("user-input").value = makeRandomID();
}

function makeRandomID() {
  return `JSClient-${Math.floor(Math.random() * 9000 + 1000)}`;
}

function pageReady() {
  initHTML();

  acquireUserMedia({
    video: true,
    audio: true,
  });
}
