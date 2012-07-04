#!/bin/bash
for i in `/usr/bin/find ./ -name \*.cpp -o -name \*.hpp -o -name \*.h`
do
  #name=`/bin/grep 'include \"' $i | /usr/bin/awk -F "\"" {'print $2'}`
  /bin/grep 'include \"' $i | /bin/grep -v "pcl"
 
#  if [ "$name" != "" ]
#  then 
#    for j in $name
#    do
#      new_name=`/bin/echo $j | /bin/sed -r "s%\/%\\\\\/%g"`
#
#      /bin/sed -Ei 's/\#include \"'$new_name'\"/\#include \<'$new_name'\>/g' $i
#      echo "Changing #include \"$new_name\" into #include <$new_name>..."
#    done
#  fi
done


