VERSION=1.6.11

all:
		wget http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-${VERSION}-src.zip
		unzip flann-${VERSION}-src.zip
		#patch -p0 < cpack.patch
		cd flann-${VERSION}-src && mkdir -p build && cd build && cmake -DBUILD_C_BINDINGS=false -DBUILD_PYTHON_BINDINGS=false -DBUILD_MATLAB_BINDINGS=false -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr .. && \
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
		rm -rf flann* *~

