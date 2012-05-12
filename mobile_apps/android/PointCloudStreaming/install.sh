#!/bin/bash

source tools.sh

$ADB -d install -r $build_dir/bin/PointCloudStreaming-debug.apk
