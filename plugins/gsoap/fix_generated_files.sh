#!/bin/bash

for i in $(grep -rl '(,' generated/); do sed -i 's/(,/(/' $i; done

