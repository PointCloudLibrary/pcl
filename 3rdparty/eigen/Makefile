VERSION=3.0.0
all: eigen
		mkdir -p eigen/build_dir
		patch -p0 < cpack.patch
		cd eigen/build_dir && cmake .. -DCMAKE_INSTALL_PREFIX=/usr .. && make package && \
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


eigen:
		hg clone https://bitbucket.org/eigen/eigen/ -r ${VERSION}

clean:
		rm -rf eigen Eigen* *~
