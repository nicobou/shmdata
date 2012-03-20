#!/bin/bash
FILES=`ls shmdata/*.{h,c} examples/*.{c,cpp}`

# reindent
indent -gnu -sob $FILES
#remove trailing spaces
sed -i "s/[ \t]*$//" $FILES

