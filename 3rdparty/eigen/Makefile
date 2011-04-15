VERSION=3.0.0
all: eigen
	mkdir -p eigen/build_dir
	patch -p0 < cpack.patch
	cd eigen/build_dir && cmake .. && make package && \
		cp *.deb ../../ && \
		cp *.rpm ../../ && \
		cp *.exe ../../

eigen:
	hg clone https://bitbucket.org/eigen/eigen/ -r ${VERSION}

clean:
	rm -rf eigen Eigen* *~
