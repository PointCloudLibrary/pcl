VERSION=1.6.8

all:
	wget http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-${VERSION}-src.zip
	unzip flann-${VERSION}-src.zip
	patch -p0 < cpack.patch
	cd flann-${VERSION}-src && mkdir -p build && cd build && cmake -DBUILD_C_BINDINGS=false -DBUILD_PYTHON_BINDINGS=false -DBUILD_MATLAB_BINDINGS=false -DCMAKE_BUILD_TYPE=RELEASE .. && BUILD_C_BINDINGS=true BUILD_MATLAB_BINDINGS=false make package && \
		cp *.deb ../../ && \
		cp *.rpm ../../

clean:
	rm -rf flann* *~

