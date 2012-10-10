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
# NE10 Library : runperf.sh
#

# NOTE: the following string comparisons differ between BASH and SH
if [ ! -n "$1" ]; then exit; fi
if [ ! -n "$2" ]; then exit; fi
if [ ! -e "./$1.test_r.ex" ]; then exit; fi
./$1.test_r.ex 0
OP_COUNT=$?
IMPL_COUNT=3
ITERATIONS=$2
PERF_CMD="perf stat -e cycles,instructions,cache-references,cache-misses,branches,branch-misses,bus-cycles,cpu-clock,task-clock,faults,minor-faults,major-faults,context-switches,migrations,alignment-faults,emulation-faults -x,"
rm res_*_$1_*.txt
for o in $(seq $OP_COUNT)
do
  ./$1.test_r.ex $o 0 $ITERATIONS
  RET=$?
  if [ "$RET" -ne "0" ]; then
     echo " SEND MAIL ~~ ERROR: Unit [$1] operation [$o] has returned with error code $RET...";
     #continue; # if one of the operations in a unit has a mismatching implementation it doesnt mean that all other op's would do too
     # dont skip the operation, try different implementations
     if [ "$RET" -eq "10" ]; then
        exit $RET;
     fi
  fi
  for i in $(seq $IMPL_COUNT)
  do
     ./$1.test_r.ex $o $i $ITERATIONS 1>/dev/null 2>/dev/null
     RET=$?
     if [ "$RET" -ne "0" ]; then
        echo "ERROR;./$1.test_r.ex $o $i $ITERATIONS $RET"
        exit $RET;
     else
        STDOUT_FILE="res_std_"$1_$o"_"$i"_"$ITERATIONS".txt";
        STDERR_FILE="res_err_"$1_$o"_"$i"_"$ITERATIONS".txt";
# Uncomment and use the following three lines if you would like to see the output from perf
#        echo "$STDOUT_FILE" > $STDOUT_FILE;
#        echo "$STDERR_FILE" > $STDERR_FILE;
#        $PERF_CMD ./$1.test_r.ex $o $i $ITERATIONS 1>>$STDOUT_FILE 2>>$STDERR_FILE;
     fi
  done
done
