#!/bin/bash

source tools.sh
export ANDROID=/home/andrey/android-sdks/tools/android
$ANDROID update project --name KiwiViewer --path $app_dir --target android-16
