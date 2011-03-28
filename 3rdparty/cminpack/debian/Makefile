# Author: Suat Gedikli
# Date:   25. March 2011

ARCHITECTURE=$(shell uname -m)
ifeq (${ARCHITECTURE},x86_64)
        ARCH=amd64
else 
        ARCH=i386
endif

DISTRO=$(shell lsb_release -sc)
VERSION=1.1.3
CMINPACK_PACKAGE_NAME=cminpack-$(VERSION)~$(DISTRO)_$(ARCH)

all : $(CMINPACK_PACKAGE_NAME).deb

cminpack_lib: cminpack/build/cmake/cminpack.pc
	cd cminpack && mkdir -p build && cd build && cmake -DUSE_FPIC=ON -DSHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMINPACK_LIB_INSTALL_DIR="." .. && make && cd ../../../../
	
cminpack/build/cmake/cminpack.pc: cminpack
	mkdir -p cminpack/build

cminpack: cminpack-${VERSION}.tar.gz
	tar -xvzf cminpack-${VERSION}.tar.gz
	mv cminpack-${VERSION} cminpack
	
cminpack-${VERSION}.tar.gz : 
	wget http://devernay.free.fr/hacks/cminpack/cminpack-${VERSION}.tar.gz

debian: $(CMINPACK_PACKAGE_NAME).deb

$(CMINPACK_PACKAGE_NAME).deb: cminpack_lib ./CONTROL/cminpack_control ./CONTROL/cminpack.pc
	mkdir -p $(CMINPACK_PACKAGE_NAME)/DEBIAN
	mkdir -p $(CMINPACK_PACKAGE_NAME)/usr/include/cminpack
	mkdir -p $(CMINPACK_PACKAGE_NAME)/usr/lib/pkgconfig
	cp -r ./cminpack/*.h $(CMINPACK_PACKAGE_NAME)/usr/include/cminpack/
	cp -r ./cminpack/build/*.so $(CMINPACK_PACKAGE_NAME)/usr/lib/
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/cminpack_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(CMINPACK_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/cminpack.pc > $(CMINPACK_PACKAGE_NAME)/usr/lib/pkgconfig/cminpack.pc
	@dpkg-deb -b $(CMINPACK_PACKAGE_NAME)
	
clean:
	rm -rf cminpack*

