## Negotiation
Once a call has been setup with the signaling server, the peers must 
negotiate SDP and ICE candidates with each other.

The calling side must create an SDP offer and send it to the peer as a JSON object:

```json
{
    "sdp": {
                "sdp": "o=- [....]",
                "type": "offer"
    }
}
```

The callee must then reply with an answer:

```json
{
    "sdp": {
                "sdp": "o=- [....]",
                "type": "answer"
    }
}
```

ICE candidates must be exchanged similarly by exchanging JSON objects:


```json
{
    "ice": {
                "candidate": ...,
                "sdpMLineIndex": ...,
                ...
    }
}
```

These structures are as specified by WebRTC. Read the WebRTC spec for more 
information on that.
