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
    sed 's/Francois/François/' | \
    sed 's/François/François/' | \
    sed 's/Jeremie Soria/Jérémie Soria/' | \
    sed 's/Marie-Eve$/Marie-Eve Dumas/' | \
    sed 's/Nina/Nicolas Bouillot/' | \
    sed 's/nicolas/Nicolas Bouillot/' | \
    sed 's/ubald/François Ubald Brien/' | \
    sed 's/vlaurent/Valentin Laurent/' | \
    sed 's/Michal Seta/Michał Seta/' | \
    sed 's/OpSocket$/Aurélien Perronneau (OpSocket)/' | \
    sed 's/Aurélien$/Aurélien Perronneau (OpSocket)/' | \
    grep -v metalab | \
    grep -v 4d3d3d3 | \
    sort | \
    uniq -c | sort -bgr | \
    sed 's/\ *[0-9]*\ /\* /' > ${AUTHORS}

