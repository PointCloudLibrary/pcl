#!/bin/bash

source tools.sh

cd $build_dir
make || exit

cd $app_dir
$ANT -buildfile $app_dir/build.xml -Dbuilddir=$build_dir debug || exit
