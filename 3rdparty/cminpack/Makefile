VERSION=1.1.3

all:
	wget http://devernay.free.fr/hacks/cminpack/cminpack-${VERSION}.tar.gz
	tar -xvzf cminpack-${VERSION}.tar.gz
	patch -p0 < cpack.patch
	cd cminpack-${VERSION} && mkdir -p build && cd build && cmake -DUSE_FPIC=ON -DSHARED_LIBS=ON -DBUILD_EXAMPLES=OFF -DCMINPACK_LIB_INSTALL_DIR="." .. && \
		make package && \
		cp *.deb ../../ && \
		cp *.rpm ../../

clean:
	rm -rf cminpack* CMINPACK* *~

