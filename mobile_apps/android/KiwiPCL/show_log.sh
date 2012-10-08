#!/bin/bash

source tools.sh

$ADB logcat KiwiViewerActivity:D KiwiViewer:D *:s
