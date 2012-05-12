# Edit the following variables for your environment:

ANT=`which ant`
ADB=`which adb`
ANDROID=`which android`
CMAKE=`which cmake`
VES_SOURCE_DIR=/source/ves/ves
VES_BUILD_DIR=/source/ves/build




#############################################
# These don't need to be modified
app_dir=$(cd $(dirname $0) && pwd)
build_dir=$app_dir
cmakeexternals=$VES_BUILD_DIR/CMakeExternals
source_dir=$VES_SOURCE_DIR
set -x
