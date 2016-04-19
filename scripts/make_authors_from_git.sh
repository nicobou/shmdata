#!/bin/bash

AUTHORS="../AUTHORS.md"
if ! [ -f "$AUTHORS" ]
then
    AUTHORS="./AUTHORS.md"
    if ! [ -f "$AUTHORS" ]
    then
	echo "no authors file found, exiting"
	exit
    fi
fi
   
   # order by number of commits
git log --format='%aN' | \
    sed 's/Francois/François/' | \
    sed 's/nicolas/Nicolas Bouillot/' | \
    sed 's/Nicolas Bouillot/Nicolas Bouillot \(lead programmer\)/' | \
    grep -v metalab | \
    sort | \
    uniq -c | sort -bgr | \
    sed 's/\ *[0-9]*\ /\* /' > ${AUTHORS}

