# Author: Suat Gedikli
# Date:   25. March 2011

ARCHITECTURE=$(shell uname -m)
ifeq (${ARCHITECTURE},x86_64)
        ARCH=amd64
else 
        ARCH=i386
endif

DISTRO=$(shell lsb_release -sc)
VERSION=1.6.8
FLANN_PACKAGE_NAME=flann-$(VERSION)~$(DISTRO)_$(ARCH)

all : $(FLANN_PACKAGE_NAME).deb

FLANN_lib: flann/build/cmake/flann.pc
	cd flann && mkdir -p build && cd build && cmake -DBUILD_C_BINDINGS=false -DBUILD_PYTHON_BINDINGS=false -DBUILD_MATLAB_BINDINGS=false -DCMAKE_BUILD_TYPE=RELEASE .. && BUILD_C_BINDINGS=true BUILD_MATLAB_BINDINGS=false make
	
flann/build/cmake/flann.pc: flann
	mkdir -p flann/build

flann: flann-${VERSION}-src
	mv flann-${VERSION}-src flann
	
flann-${VERSION}-src : flann-${VERSION}-src.zip
	unzip flann-${VERSION}-src.zip
	
flann-${VERSION}-src.zip : 
	wget http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-${VERSION}-src.zip

debian: $(FLANN_PACKAGE_NAME).deb

$(FLANN_PACKAGE_NAME).deb: FLANN_lib ./CONTROL/flann_control ./CONTROL/flann.pc
	mkdir -p $(FLANN_PACKAGE_NAME)/DEBIAN
	mkdir -p $(FLANN_PACKAGE_NAME)/usr/include/flann/util
	mkdir -p $(FLANN_PACKAGE_NAME)/usr/include/flann/nn
	mkdir -p $(FLANN_PACKAGE_NAME)/usr/lib/pkgconfig
	cp -r ./flann/src/cpp/flann/*.h $(FLANN_PACKAGE_NAME)/usr/include/flann/
	cp -r ./flann/src/cpp/flann/*.hpp $(FLANN_PACKAGE_NAME)/usr/include/flann/
	cp -r ./flann/src/cpp/flann/algorithms $(FLANN_PACKAGE_NAME)/usr/include/flann/
	cp -r ./flann/src/cpp/flann/io $(FLANN_PACKAGE_NAME)/usr/include/flann/
	cp -r ./flann/src/cpp/flann/nn/*.h $(FLANN_PACKAGE_NAME)/usr/include/flann/nn/
	cp -r ./flann/src/cpp/flann/util/*.h $(FLANN_PACKAGE_NAME)/usr/include/flann/util/
	cp -r ./flann/src/cpp/flann/util/*.hpp $(FLANN_PACKAGE_NAME)/usr/include/flann/util/
	cp -r ./flann/lib/* $(FLANN_PACKAGE_NAME)/usr/lib/
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/flann_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(FLANN_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/flann.pc > $(FLANN_PACKAGE_NAME)/usr/lib/pkgconfig/flann.pc
	@dpkg-deb -b $(FLANN_PACKAGE_NAME)
	
clean:
	rm -rf flann*

