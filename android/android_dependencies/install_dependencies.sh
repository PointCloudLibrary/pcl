#~/bin/bash
PCL_ANDROID_DIR=`pwd`

echo "Get the source code for android-cmake first

"
hg clone https://code.google.com/p/android-cmake/

echo "


Done ...
"


echo "Setting the android-cmake environment variables
"
cd android-cmake
ANDROID_CMAKE_DIR=`pwd`

echo "
" >> $HOME/.bashrc
echo export ANDROID_CMAKE=$ANDROID_CMAKE_DIR >> $HOME/.bashrc
echo export ANDTOOLCHAIN=$ANDROID_CMAKE/toolchain/android.toolchain.cmake >> $HOME/.bashrc
echo "alias android-cmake='cmake -DCMAKE_TOOLCHAIN_FILE=$ANDTOOLCHAIN '" >> $HOME/.bashrc

OS=`uname -s`
if [ "$OS"="Darwin" ]; then
	echo "
	" >> $HOME/.profile
	echo source $HOME/.bashrc >> $HOME/.profile
fi

source $HOME/.bashrc


echo "Done ...
"


echo "Installing Boost
"
cd common-libs/boost
sh get_boost.sh
mkdir build
cd build
android-cmake ..
make -j4
sudo make install

echo "Done ...

"


echo "Installing Eigen
"

cd ../../eigen
sh get_eigen.sh

echo "Applying ugly hack to pass the standard math library test of Eigen - does not work on the arm compiler"
patch -p1 -i $PCL_ANDROID_DIR/FindStandardMathLibrary.cmake.patch eigen-android/cmake/FindStandardMathLibrary.cmake
cd eigen-android
mkdir build
cd build
android-cmake ..
make -j4
sudo make install

echo "Done ...

"

echo "Installing FLANN
"

cd $PCL_ANDROID_DIR
git clone git://github.com/mariusmuja/flann.git
cd flann
mkdir build
cd build
android-cmake -DBUILD_MATLAB_BINDINGS=0 -DBUILD_PYTHON_BINDINGS=0 -DBUILD_CUDA_LIB=0 ..
make -j4
sudo make install
