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
# NE10 Library : cleanall.sh
#

PRODUCT_NAME=NE10

rm *.ex *.a *.o *.so
rm res_*.txt
rm .*.swp
rm .exp.tmp
rm testlog.txt
for dir in `find * -maxdepth 0 -type d -name "${PRODUCT_NAME}_*"`; do rm -rf $dir; done;
rm -rf ./java
for fl  in `find * -maxdepth 0 -type f -name "${PRODUCT_NAME}_*.tgz"`; do rm -rf $fl; done;
if [ "$CLS" != "0" ]; then
 clear
 echo
 ls -la --color=auto
 echo
fi
echo

