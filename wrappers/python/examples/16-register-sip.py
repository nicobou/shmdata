from pyquid import Switcher, InfoTree

import json

host = "my-host"
user = 'my-user'
pswd = "my-pass"

sw = Switcher("test")
sip_config = {
    "port": "5060",
    "user": user,
    "pass": pswd,
    "stun": host,
    "turn": host,
    "turn_user": user,
    "turn_pass": pswd
}

sip_quid = sw.create(
    "sip", "sip_quid", InfoTree(json.dumps(sip_config)))

address = f"{user}@{host}"
reg = sip_quid.invoke("register", [address, pswd])
print(reg)

sip_quid.invoke("unregister")
