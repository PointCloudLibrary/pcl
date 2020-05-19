#! /usr/bin/env bash

new_version=$(git tag | sort -rV | head -1 | cut -d- -f 2).99

# Mac users either use gsed or add "" after -i
if ls | grep README -q; then
    sed -i "s,VERSION [0-9.]*),VERSION ${new_version})," CMakeLists.txt
else
    echo "Don't think this is the root directory" 1>&2
    exit 4
fi
