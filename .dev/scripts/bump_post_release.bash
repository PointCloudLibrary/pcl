#! /usr/bin/env bash

# Mac users either use gsed or add "" after -i
if ls | grep README -q; then
    # Just add .99 at the end of the version
    sed -i "s,PCL VERSION [0-9.]*,&.99," CMakeLists.txt
else
    echo "Don't think this is the root directory" 1>&2
    exit 4
fi
