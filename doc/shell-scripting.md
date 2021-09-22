Shell Scripting
=======

Switcher and scenic can be scripted using the soap quiddity. Features include creation/deletion of quiddities, set/get properties.

## Example with switcher

Run the switcher server as follows:
```
switcher -d
```

Then you can send commands to the server using the command line tool switcher-ctrl. The following commands will create a videotestsrc and connect its Shmdata into a video window:
```
# create the quiddities
switcher-ctrl -C videotestsrc vid
switcher-ctrl -C glfwin win
# start the video with the "started" property set to true (will activate the video Shmdata)
switcher-ctrl -s vid started true
# connect them both
switcher-ctrl -o win vid

# change the video pattern :
switcher-ctrl -s vid pattern 18
switcher-ctrl -s vid pattern "Checkers 1px"
```

More options are available, check -h option for both switcher and switcher-ctrl commands.

## Scripting scenic

Run scenic with soap enabled, and then send commands as it can be done with switcher: 
```
scenic -s
```

## Remote control

Use the switcher-ctrl --server option for sending to a remote server
```
switcher-ctrl --server  http://localhost:27182 -C videotestsrc vid
```

Where localhost can be replaced by the machine name or IP address of the remote server, and 27182 the port used by the server for receiving soap commands.

## Save your JSON data associated with quiddities
Run switcher and apply the following commands in another terminal: 
```
switcher-ctrl -C dummy dum
# add information in the user tree of a quiddity :
switcher-ctrl -a dum mon.test.val1 float 8.9
switcher-ctrl -a dum mon.test.val2 string "Coucue * ^%"
switcher-ctrl -a dum mon.test.val3 bool false
# display the user tree (JSON) :
switcher-ctrl -u dum
# remove a branch:
switcher-ctrl -r dum mon.test.val1
 
# save the session :
switcher-ctrl -w /tmp/montest.switcher
 
# quit switcher
killall switcher
 
# load the previously saved session when running a new switcher server :
switcher -l /tmp/montest.switcher -d
 
# read the user tree previously constituted
switcher-ctrl -u dum
```
