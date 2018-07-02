#!/usr/bin/env python3

import sys
assert sys.version_info >= (3, 5)

import os
import signal
import time
import json

from pathlib import Path

sys.path.insert(0, '/usr/local/lib/python3/dist-packages')
import pyquid


# optional ctrl-c handler
def sigint_handler(signum, frame):
    print('Will soon exit...')
    sys.exit()


signal.signal(signal.SIGINT, sigint_handler)

config_file = "config.json"

# sample config.json file:
"""
{
  "sip" : {
    "port" : "5060",
    "console_log_level": 2,
    "log_level": 4,
    "log_filename": "log.fifo",
    "user" : "user@server.com",
    "pass" : "mypass",
    "stun": "mystun.com",
    "turn": "myturn.com",
    "turn_user": "user",
    "turn_pass": "pass"
  }
}
"""

sw = pyquid.Switcher(name="pyquid", debug=True, config=config_file)

qrox = sw.create(type="sip", name="sip")
sip = qrox.quid()

# uncomment if no config is provided, or for options not in the config
#~ user = "user@server.com"
#~ password = "pass"
#~ stun_server = "mystun.com"
#~ turn_server = "myturn.com"
#~ turn_user = "user"
#~ turn_password = "pass"
#~ sip.invoke_str("set_stun_turn", [stun_server, turn_server, turn_user, turn_password])
#~ sip.invoke_str("register", [user, password])

# specify buddies; repeat for each buddy to monitor
buddy = "mybuddy@server.com"
buddy_name = "my best buddy"
sip.set("mode", "authorized contacts")
sip.invoke("add_buddy", [buddy, buddy_name])


# Check if there a log file specified in the config file
logfile = None
with open(config_file) as handle:
    config_dict = json.loads(handle.read())
    if config_dict \
            and 'sip' in config_dict \
            and 'log_filename' in config_dict['sip']:
        log_filename = config_dict['sip']['log_filename']
    if log_filename:  # "touch" file
        logfile = open(log_filename, "r")


# Follow allows for live reading of logs, like tail -f
# from : http://www.dabeaz.com/generators/follow.py

def follow(thefile):
    thefile.seek(0, 2)
    while True:
        line = thefile.readline()
        if not line:
            time.sleep(0.1)
            continue
        yield line


# infinite loop, so the script stays open (ctl-c will exit the loop)
# it includes a simple watchdog mechanism (to check optional log file)
while True:
    if logfile:
        loglines = follow(logfile)
        for line in loglines:
            # check for interesting event(s) in the log file
            if '500 Unhandled by dialog usages' in line:
                print(line)
                # handle error, for example:
                # restart SIP client to keep presence active
    else:
        time.sleep(0.1)
