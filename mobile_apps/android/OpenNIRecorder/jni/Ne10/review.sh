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
# NE10 Library : review.sh
#

BRANCH=$1

BASE=${2-"master"}

if [ "$BRANCH" = "" ]; then
  echo "Usage: review.sh <branch to review> [parent branch]"
  exit
else

  LABEL=`echo $1 | perl -pe '$_ =~ /dev\/([a-zA-Z0-9]+)\/(.+)/;$_=$2'`
  GLUSER=`echo $1 | perl -pe '$_ =~ /dev\/([a-zA-Z0-9]+)\/(.+)/;$_=$1'`

  NEWBRANCH="staging/$GLUSER/$LABEL"

  echo "Pushing $BRANCH from $BASE for review as $NEWBRANCH"

  git branch $NEWBRANCH $BASE
  git push origin $NEWBRANCH
  git checkout $NEWBRANCH
  git rebase $BRANCH
  git push origin $NEWBRANCH

fi

