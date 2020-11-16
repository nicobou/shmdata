#!/bin/bash

# the script will fail if one command fails
set -e

# run a switcher
switcher --debug&
ret=$!
sleep 3

# query information to the switcher
switcher-ctrl --list-classes
switcher-ctrl --list-quiddities
switcher-ctrl --classes-doc
switcher-ctrl --class-doc dummy

# add a quiddity named foo 
switcher-ctrl --create-quiddity dummy foo

# set get the foo property
switcher-ctrl --set-prop foo int_ 3
switcher-ctrl --get-prop foo int_

# query information about the foo quiddity
switcher-ctrl --shmpath foo video
switcher-ctrl --print-tree foo \.property\.
switcher-ctrl --quiddities-descr
switcher-ctrl --quiddity-descr

# user tree manipulation
switcher-ctrl --graft-user-data foo .hello string world
switcher-ctrl --graft-user-data foo .yes bool true
switcher-ctrl --graft-user-data foo .month string 2
switcher-ctrl --graft-user-data foo .pi float "3.14"
switcher-ctrl --graft-user-data foo .array.1 string me
switcher-ctrl --graft-user-data foo .array.2 string you
switcher-ctrl --graft-user-data foo .array.3 string us
switcher-ctrl --tag-as-array-user-data foo .array
switcher-ctrl --graft-user-data foo .to_remove string please
switcher-ctrl --prune-user-data foo .to_remove
switcher-ctrl --print-user-data foo

# add a quiddity named bar
switcher-ctrl --create-quiddity method bar
switcher-ctrl --invoke-method bar hello_ "Ringo Star"

# save/remove foo/load
switcher-ctrl --save ./switcher_save
switcher-ctrl -D foo
switcher-ctrl --load ./switcher_save
switcher-ctrl --set-prop foo int_ 4
rm ./switcher_save

# kill the switcher
kill -s SIGQUIT $ret

exit 0
