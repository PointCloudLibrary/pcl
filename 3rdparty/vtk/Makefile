VERSION=5.6.1
UNAME := $(shell uname -s)

all: patch
		make package && \
		if test -e *.exe; \
		then cp *.exe ../../; \
		fi; \
		if test -e *.rpm; \
		then cp *.rpm ../../; \
		fi; \
		if test -e *.deb; \
		then cp *.deb ../../; \
		fi; \
		if test -e *.dmg; \
		then cp *.dmg ../../; \
		fi;

vtk:
		wget http://www.vtk.org/files/release/5.6/vtk-${VERSION}.tar.gz
		tar -xvzf vtk-${VERSION}.tar.gz

patch: vtk
ifeq ($(UNAME),Darwin)
		patch -p0 < cpack.darwin.patch && \
		cd VTK && mkdir -p build && cd build && \
		cmake .. -DCMAKE_INSTALL_PREFIX=/usr .. \
		-DCMAKE_BUILD_TYPE="Release" \
		-DBUILD_SHARED_LIBS=ON \
		-DBUILD_TESTING=OFF \
		-DVTK_USE_X=ON \
		-DVTK_USE_COCOA=OFF
else
		patch -p0 < cpack.patch
		cd VTK && mkdir -p build && cd build && \
		cmake .. -DCMAKE_INSTALL_PREFIX=/usr .. \
		-DCMAKE_BUILD_TYPE="Release" \
		-DBUILD_SHARED_LIBS=ON \
		-DBUILD_TESTING=OFF
endif

clean:
		rm -rf vtk* VTK* *~

