ANT=`which ant`
ADB=`which adb`
ANDROID=`which android`
CMAKE=`which cmake`


app_dir=$(cd $(dirname $0) && pwd)
source_dir=/home/andrey/kiwi/VES
build_dir=$app_dir

cmakeexternals=/home/andrey/kiwi/VES/Apps/Android/CMakeBuild/build/CMakeExternals

set -x
