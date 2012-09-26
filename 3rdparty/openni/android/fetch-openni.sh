#!/bin/sh

set -e

if type dos2unix > /dev/null; then
  DOS2UNIX=dos2unix
elif type fromdos > /dev/null; then
  DOS2UNIX=fromdos
else
  echo "You will need either dos2unix or fromdos for this to work."
  exit 1
fi

git clone git://github.com/OpenNI/OpenNI.git OpenNI
cd OpenNI
git checkout Stable-1.5.2.23

find . -type f '(' -name *.h -o -name *.c -o -name *.cpp -o -name *.java -o -name *.py -o -name *.sh ')' \
  -exec $DOS2UNIX {} ';'

cat ../openni-patches/series | while read patchname; do
  patch -p1 -i ../openni-patches/$patchname
done
