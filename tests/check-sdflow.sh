#! /bin/bash

sdflow -d /tmp/test &
SDFLOW=$!

sleep 1

sdcrash -v -d -q -n 100 /tmp/test
RET=$?

kill $SDFLOW

exit $RET

