#!/bin/bash

# the script will fail if one command fails
set -e

# print version
switcher --version

# list quiddity kinds
switcher --list-kinds

# print kinds documentation, JSON-formated
switcher --kinds-doc

# print documentation for a given kind (here the "dummy" quiddity kind)
switcher --kind-doc dummy

# run a quiet switcher
switcher --quiet --extra-plugin-dir ../plugins/gsoap/&
ret=$!
sleep 1
kill -s SIGQUIT $ret

# run switcher with several parameters
switcher --server-name checkswitcher --port-number 23232 --debug --extra-plugin-dir ../plugins/gsoap/ &
ret=$!
sleep 1
kill -s SIGQUIT $ret

exit 0
