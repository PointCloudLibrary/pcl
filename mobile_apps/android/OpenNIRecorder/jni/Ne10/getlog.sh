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
# NE10 Library : getlog.sh
#

echo "NE10 NIGHTLY BUILD SCRIPT"
echo "(C) 2011, ARM Ltd."
date

echo
echo
echo -e "\033[4mSYSTEM:\033[0m"
uname -a
cat /proc/cpuinfo

echo
echo
echo -e "\033[4mINSTALLED TOOLS:\033[0m"
echo "git:"
if [ "`which git`" = "" ]; then
 echo "fatal: 'git' is not installed on this system" 1>&2
 exit 1
fi
git --version | paste -s -d ';' -
echo
echo "gcc:"
if [ "`which gcc`" = "" ]; then
 echo "fatal: 'gcc' is not installed on this system" 1>&2
 exit 1
fi
gcc --version | paste -s -d ';' -
echo
echo "as:"
if [ "`which as`" = "" ]; then
 echo "fatal: 'as' is not installed on this system" 1>&2
 exit 1
fi
as --version | paste -s -d ';' -
echo
echo "ar:"
if [ "`which ar`" = "" ]; then
 echo "fatal: 'ar' is not installed on this system" 1>&2
 exit 1
fi
ar --version | paste -s -d ';' -
echo
echo
echo "perl:"
if [ "`which perl`" = "" ]; then
 echo "fatal: 'perl' is not installed on this system" 1>&2
 exit 1
fi
perl --version | paste -s -d ';' -

echo
if [ -e .git ]; then
 echo
 echo -e "\033[4mCURRENT 'git' CONFIGURATION:\033[0m"
 git config -l
fi

echo
echo
echo -e "\033[4mCURRENT USER AND PATH:\033[0m"
echo `whoami` "@" `pwd`

echo
echo
echo -e "\033[4mENVIRONMENT VARIABLES:\033[0m"
echo
echo "PATH = " $PATH
echo
echo "LD_LIBRARY_PATH = " $LD_LIBRARY_PATH


echo
if [ -e .git ]; then
echo
echo -e "\033[4mCURRENT GIT/SOURCE STATUS:\033[0m"
  git show
fi


