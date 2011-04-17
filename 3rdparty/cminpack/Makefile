VERSION=1.1.3

all:
		wget http://devernay.free.fr/hacks/cminpack/cminpack-${VERSION}.tar.gz
		tar -xvzf cminpack-${VERSION}.tar.gz
		patch -p0 < cpack.patch
		cd cminpack-${VERSION} && mkdir -p build && cd build && cmake -DUSE_FPIC=ON -DSHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMINPACK_LIB_INSTALL_DIR="." .. && \
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

clean:
		rm -rf cminpack* CMINPACK* *~

