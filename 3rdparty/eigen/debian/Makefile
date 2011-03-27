# Author: Suat Gedikli
# Date:   25. March 2011

all : eigen_lib debian

eigen_lib: eigen/build_dir/eigen3.pc
	
eigen/build_dir/eigen3.pc: eigen
	mkdir -p eigen/build_dir
	cd eigen/build_dir && cmake ..

eigen:
	hg clone  https://bitbucket.org/eigen/eigen/ -r 3.0.0

ARCHITECTURE=$(shell uname -m)
ifeq (${ARCHITECTURE},x86_64)
        ARCH=amd64
else 
        ARCH=i386
endif

DISTRO=$(shell lsb_release -sc)
VERSION=$(shell grep Version eigen/build_dir/eigen3.pc | sed "s|[^[:digit:].]||g")

EIGEN_PACKAGE_NAME=Eigen3-$(VERSION)~$(DISTRO)_$(ARCH)

debian: $(EIGEN_PACKAGE_NAME).deb

$(EIGEN_PACKAGE_NAME).deb: ./CONTROL/eigen_control ./CONTROL/eigen3.pc eigen/build_dir/eigen3.pc
	mkdir -p $(EIGEN_PACKAGE_NAME)/DEBIAN
	mkdir -p $(EIGEN_PACKAGE_NAME)/usr/include/Eigen
	mkdir -p $(EIGEN_PACKAGE_NAME)/usr/lib/pkgconfig
	cp -r ./eigen/Eigen/* $(EIGEN_PACKAGE_NAME)/usr/include/Eigen/
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/eigen_control | sed s/__ARCHITECTURE__/$(ARCH)/ > $(EIGEN_PACKAGE_NAME)/DEBIAN/control
	@sed s/__VERSION__/${VERSION}~${DISTRO}/ ./CONTROL/eigen3.pc > $(EIGEN_PACKAGE_NAME)/usr/lib/pkgconfig/eigen3.pc
	@dpkg-deb -b $(EIGEN_PACKAGE_NAME)
	
clean:
	rm -rf $(EIGEN_PACKAGE_NAME)*
	rm -rf eigen
