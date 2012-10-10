#!/bin/sh
#
#  Copyright 2011-12 ARM Limited
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

#
# NE10 Library : removetabs.sh
#
#!/bin/bash

# This script removes tab characters in files and replaces them with
#  the right number of spaces. It also removes trailing whitespaces.

# remove trailing whitespaces
LSw=`grep -lsri --exclude="Makefile" --exclude-dir=".git" '\s$' .`;
for flw in $LSw
do
    echo "HAS SPACES: " $flw; # just to see a list of the files that include unwanted tabs
    perms=`stat -c '%a' $flw`;
    sed 's/[ \t]*$//gi' $flw > .exp.tmp;
    sync;
    # rename the file to the original file
    mv .exp.tmp $flw;
    chmod $perms $flw;
    sync;
done

# remove tabs
chtab=$'\t'; # only works in bash but not in sh
LSt=`grep -lrsi --exclude="Makefile" --exclude-dir=".git" "$chtab" .`;
for flt in $LSt
do
    echo "HAS TABS: " $flt; # just to see a list of the files that include unwanted tabs
    perms=`stat -c '%a' $flt`;
    # remove tabs
    expand $flt > .exp.tmp;
    sync;
    # rename the file to the original file
    mv .exp.tmp $flt;
    chmod $perms $flt;
    sync;
done

