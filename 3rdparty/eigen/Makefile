all: eigen
	mkdir -p eigen/build_dir
	patch -p0 < cpack.patch
	cd eigen/build_dir && cmake .. && make package
	cp *.rpm ../../ 
	cp *.deb ../../ 
	cp *.exe ../../ 

eigen:
	hg clone https://bitbucket.org/eigen/eigen/ -r 3.0.0

clean:
	rm -rf eigen
