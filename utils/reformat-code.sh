#!/bin/bash
FILES=`ls shmdata/*.{h,c} examples/*.{c,cpp} tests/*.c`

# reindent
indent -gnu -sob $FILES
#remove trailing spaces
sed -i "s/[ \t]*$//" $FILES
# append one blank line at the end of each file
for f in $FILES
do
    echo "" >> $f
done

