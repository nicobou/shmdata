Protocol Mapper
=======

The protocol-mapper quiddity provides the mapping of properties on OSC and/or HTTP messages. This is done through definition of a set of commands in a JSON file.

The OSC protocol is already implemented and an example JSON protocol-osc.json is available in plugins/protocol-mapper/ in switcher. Here is an excerpt of the file that helps us comment it:
```
{
  "Protocol" : "osc",
  "url" : "osc.udp://localhost:3819",
  "continuous" : true,
  "commands": {
    "int32" : {
      "type" : "i",
      "value" : 45,
      "name": "int32",
      "path" : "/what/a/pretty/path",
      "descr": "sends an int32"
    }
}
```


Here is a description of each key:

* Protocol (mandatory): can currently be osc or curl, indicates which protocol implementation will be used.
* URL (optional, only for OSC protocol): default URL to which generated OSC messages will be sent. Can be modified in the quiddity afterwards.
* Continuous (optional): set to "true" if you want to periodically send some messages (the period will then be configurable in the quiddity).
* Commands (mandatory): contains all the custom commands, needed to create the properties and create the messages.

The command fields are as follows:
* type (optional): if no type is provided, an empty message will be sent, can be useful and even mandatory for some clients (some Ardour commands for instance). If a type is provided, it must be valid or the message won't be sent. Possible options are `h` (int64_t), `i` (int32_t), `f` (float), `d` (double) and `s` (string)
* value (optional): value of the message, must fit the provided type. Will be ignored if the type is empty.
* name (Optional but recommended): name of the property in the scenic inspector
* descr (optional): description of the property in the inspector
* path (optional): path to use to send the OSC message will be empty if no path is provided, which can be correct.
* continuous (optional): if set to "true" and the top-level option also set to "true" will periodically send the message, the period will be a property ranging from 50 to 10000 ms.

Here is a screenshot of the example JSON for the OSC protocol.

![Property to OSC mapper in scenic (example JSON)](mapper_prop_osc.png?raw=true "Protocol mapper in scenic")

Example with Ardour OSC protocol
--------------------------------

You can see on the example above that there is an ardour restart property. This property is simply an empty message to "/ardour/goto_start" on the local port 3819. See the description of the command:
osc_ardour_command
```
"ardour_goto_start" : {
  "path" : "/ardour/goto_start",
  "name" : "ardour restart",
  "descr": "sends restart signal to listening ardour instances"
}
```

Using the controls described on this page, it is possible to control Ardour with this quiddity. Keep in mind that you have to prefix all the provided paths with "/ardour" and that the commands without value have to be empty messages, just like "/goto_start".

