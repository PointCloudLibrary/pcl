#! /usr/bin/env bash

if [ $# != 1 ]; then
    echo "Need (only) the release version" 1>&2
    exit 3
fi

# Mac users either use gsed or add "" after -i
new_version="$1"
if ls | grep README -q; then
    sed -i "s,[0-9]\+\.[0-9]\+\.[0-9]\+,${new_version}," README.md
    sed -i "s,VERSION [0-9.]*),VERSION ${new_version})," CMakeLists.txt
else
    echo "Don't think this is the root directory" 1>&2
    exit 4
fi
