#!/bin/sh

set -e

. ./normalize-eol.sh

rm -rf OpenNI 2> /dev/null

echo 'Downloading OpenNI...'

mkdir OpenNI
cd OpenNI
wget -vO- http://github.com/OpenNI/OpenNI/tarball/Stable-1.5.2.23 | tar -xz --strip-components=1

echo 'Normalizing EOL characters to LF...'

normalize_eol

echo 'Patching...'

cat ../openni-patches/series | while read patchname; do
  case "$patchname" in
    \#*)
      ;; # ignore comments
    *)
      patch -p1 -i ../openni-patches/$patchname
      ;;
  esac
done

echo 'Done.'