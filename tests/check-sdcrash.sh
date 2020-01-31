#! /bin/bash

for i in $(seq 10); do
sdcrash -v -d -n 10 /tmp/test-sdcrash
RET=$?
if [ ! RET ] ; then exit 1; fi
done

exit $RET

