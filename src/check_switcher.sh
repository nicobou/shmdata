#!/bin/bash

# the script will fail if one command fails
set -e

# print version
switcher --version

# list quiddity classes
switcher --list-classes

# print classes documentation, JSON-formated
switcher --classes-doc

# print documentation for a given class (here the "dummy" quiddity type)
switcher --class-doc dummy

# run a quiet switcher
switcher --quiet &
ret=$!
sleep 1
kill -s SIGQUIT $ret

# run switcher with several parameters
switcher --server-name checkswitcher --port-number 23232 --debug --extra-plugin-dir . &
ret=$!
sleep 1
kill -s SIGQUIT $ret

exit 0
