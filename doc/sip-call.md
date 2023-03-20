Make a multichannel call with the sip quiddity   
=======

## Requirements

You need two SIP credentials in order to connect to the SIP/STUN/TURN servers:
* a SIP user and password, for instance switcher@mondomaine.com and mypassword
* a SIP server address, for instance sip.mondomaine.com
* a STUN server address, for instance stun.mondomaine.com
* a TURN server address, for instance turn.mondomaine.com
* a STUN/TURN user and password, for instance forward@mondomaine.com and ilovedogs

In the following example, the callee SIP user will be shmdata@mondomaine.com.

Note that you can install your own server following the documentation here:
https://gitlab.com/nicobou/sip-server

## Prepare the receiver

In a terminal run switcher:
```
switcher -d
```

In another terminal, prepare you sip communication:
```
switcher-ctrl -C sip sip
switcher-ctrl -s sip port 15060 # set local port to bind for sip communications
switcher-ctrl -i sip register shmdata@mondomaine.com:5060 ilovedogs
switcher-ctrl -i sip set_stun_turn stun.mondomaine.com:3478 turn.mondomaine.com:3478 shmdata ilovedogs

switcher-ctrl -s sip mode "authorized contacts"
switcher-ctrl -i sip add_buddy switcher@mondomaine.com
switcher-ctrl -i sip authorize switcher@mondomaine.com true
switcher-ctrl -t sip buddies.0
```

## Prepare the sender
In a terminal run switcher:
```
switcher -d -p 15432 -n caller

switcher-ctrl --server http://localhost:15432 -C sip sip
switcher-ctrl --server http://localhost:15432 -s sip port 5061
switcher-ctrl --server http://localhost:15432 -i sip register switcher@mondomaine.com ilovedogs
switcher-ctrl --server http://localhost:15432 -i sip set_stun_turn stun.mondomaine.com turn.mondomaine.com shmdata ilovedogs

switcher-ctrl --server http://localhost:15432 -i sip add_buddy shmdata@mondomaine.com
switcher-ctrl --server http://localhost:15432 -t sip buddies.0
```

Prepare two audio streams to send
```
switcher-ctrl --server http://localhost:15432 -C audiotestsrc aud1
switcher-ctrl --server http://localhost:15432 -s aud1 wave 0
switcher-ctrl --server http://localhost:15432 -s aud1 started true
switcher-ctrl --server http://localhost:15432 -C audiotestsrc aud2
switcher-ctrl --server http://localhost:15432 -s aud2 wave 1
switcher-ctrl --server http://localhost:15432 -s aud2 started true
```

Prepare the call, i.e., attach several audio Shmdata to the callee 
```
switcher-ctrl --server http://localhost:15432 -i sip attach_shmdata_to_contact $(switcher-ctrl --server http://localhost:15432 -p aud1 audio) shmdata@mondomaine.com true
switcher-ctrl --server http://localhost:15432 -i sip attach_shmdata_to_contact $(switcher-ctrl --server http://localhost:15432 -p aud2 audio) shmdata@mondomaine.com true
switcher-ctrl --server http://localhost:15432 -t sip buddies.0
```

Then make switcher call Shmdata:
```
switcher-ctrl --server http://localhost:15432 -i sip send shmdata@mondomaine.com
```
If you see from logs the call has been refused, you need to authorize your buddy to send you streams (authorize method of the sip quiddity). 

## Check the call is receiving
Ask the callee switcher
```
switcher-ctrl -t sip buddies # "recv_status" should be "receiving"
switcher-ctrl -e
```
The receiver has created one quiddity per received stream. Here aud2-switcher-sip-scenic-sat-qc-ca and aud-switcher-sip-scenic-sat-qc-ca.

Check data are received with sdflow
```
sdflow switcher_default_sip-nico2_rtp-aud2

```

## Warning

If you want to transmit video, you need to compress it, for instance with the nvenc or videnc quiddity. 
