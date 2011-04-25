VERSION=2011.1

all:
		wget http://www.qhull.org/download/qhull-${VERSION}-src.tgz
		tar -xvzf qhull-${VERSION}-src.tgz
		patch -p0 < cpack.patch
		cd qhull-${VERSION}/build && cmake -DBUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr .. && \
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
		rm -rf qhull* *~

